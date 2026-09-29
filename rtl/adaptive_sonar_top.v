// ============================================================================
// Adaptive Sonar Transmitter - Top Level  (v3)
// ----------------------------------------------------------------------------
//  * Reset synchroniser (async assert, sync release); ONE clock domain.
//  * Hardware sample timer: `tick` = CLK_HZ / FS_HZ (the FPGA equivalent of the
//    MCU "hardware timer"); the DDS pipeline streaming straight to the DAC
//    pins is the "DMA" -- no CPU anywhere in the sample path.
//  * Ping sequencer:  PREP -> TX -> LISTEN -> PREP ...
//      PREP   : amp_en goes high AMP_LEAD_US before the pulse so the analog
//               stage is powered and settled when the first sample arrives
//      TX     : pulse_width samples
//      LISTEN : listen_width samples (= 2R/c); amp stays on AMP_TAIL_US for
//               the filter to ring down, then switches off
//  * Low power: the DDS datapath and its BRAMs are clock-enabled only during
//    TX (+ a short pipeline flush); amp_en powers the analog front end only
//    around the pulse. Output sits at mid-scale (0 V) the rest of the time,
//    so the R2R pins stop toggling too.
//  * All parameters (and the window choice) are latched once per ping, at
//    the start of PREP: a sensor change never alters a pulse mid-flight.
//    Adaptation latency <= one ping period.
// ============================================================================

module adaptive_sonar_top #(
    parameter integer CLK_HZ      = 100_000_000,
    parameter integer FS_HZ       = 10_000_000,
    parameter integer OUT_W       = 8,
    parameter integer AMP_LEAD_US = 200,
    parameter integer AMP_TAIL_US = 50,
    // pass-through to param_controller
    parameter integer PW_MIN_US   = 1000,
    parameter integer PW_MAX_US   = 8000,
    parameter integer R_MIN_M     = 10,
    parameter integer R_MAX_M     = 150,
    parameter         SINE_FILE   = "sine_quarter.mem",
    parameter         WIN_FILE    = "window.mem",
    parameter         SS_FILE     = "soundspeed.mem",
    parameter         SAL_FILE    = "salinity.mem"
)(
    input  wire                     clk,
    input  wire                     rst,              // async assert, active high
    input  wire [7:0]               depth,
    input  wire [7:0]               turbidity,
    input  wire [7:0]               temperature,
    input  wire [7:0]               salinity,
    input  wire                     mode_override_en,
    input  wire [2:0]               mode_override,
    input  wire [2:0]               win_sel,          // 0 rect 1 Hann 2 Hamming 3 Blackman 4 Tukey(0.25)

    output wire [OUT_W-1:0]         dac_data,         // offset binary -> R2R ladder
    output wire signed [OUT_W-1:0]  sample_out,
    output wire signed [15:0]       sample16,
    output wire                     sample_valid,
    output wire                     sample_tick,
    output wire [2:0]               waveform_mode,    // controller's current decision
    output wire [2:0]               active_mode,      // mode of the ping being sent
    output wire [9:0]               fc_khz,           // controller's current centre freq
    output reg                      tx_active,
    output reg                      amp_en,
    output reg                      pulse_start,      // 1-clk strobe, scope trigger
    output wire [31:0]              dbg_freq,
    output wire [23:0]              dbg_pulse_width,
    output wire [23:0]              dbg_listen_width,
    output wire [7:0]               dbg_amplitude
);

    localparam integer SAMPLE_DIV = CLK_HZ / FS_HZ;
    localparam integer LEAD  = (AMP_LEAD_US * (FS_HZ / 1000)) / 1000 + 1;
    localparam integer TAIL  = (AMP_TAIL_US * (FS_HZ / 1000)) / 1000;
    localparam integer FLUSH = 8;                     // > DDS pipeline depth

    // geometric-sweep multiplier needs >= 4 clocks per sample
    initial if (SAMPLE_DIV < 4)
        $display("ERROR: CLK_HZ / FS_HZ must be at least 4 (is %0d)", SAMPLE_DIV);

    // ---------------- reset synchroniser ----------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sr = 2'b11;
    always @(posedge clk or posedge rst)
        if (rst) rst_sr <= 2'b11; else rst_sr <= {rst_sr[0], 1'b0};
    wire rst_s = rst_sr[1];

    // ---------------- sample-rate timer ----------------
    localparam integer DIV_W = $clog2(SAMPLE_DIV);
    reg [DIV_W-1:0] div_cnt;
    wire tick = (div_cnt == SAMPLE_DIV - 1);
    always @(posedge clk)
        if (rst_s || tick) div_cnt <= {DIV_W{1'b0}};
        else               div_cnt <= div_cnt + 1'b1;
    assign sample_tick = tick;

    // ---------------- controller ----------------
    wire [31:0]        freq_word, win_step;
    wire signed [47:0] chirp_rate;
    wire [39:0]        geo_gain;
    wire [23:0]        chip_width, pulse_width, listen_width;
    wire [7:0]         amplitude;
    wire               params_valid;

    param_controller #(
        .FS_HZ(FS_HZ), .PW_MIN_US(PW_MIN_US), .PW_MAX_US(PW_MAX_US),
        .R_MIN_M(R_MIN_M), .R_MAX_M(R_MAX_M), .SS_FILE(SS_FILE), .SAL_FILE(SAL_FILE)
    ) u_param (
        .clk(clk), .rst(rst_s),
        .depth(depth), .turbidity(turbidity), .temperature(temperature), .salinity(salinity),
        .mode_override_en(mode_override_en), .mode_override(mode_override),
        .freq_word(freq_word), .chirp_rate(chirp_rate), .geo_gain(geo_gain),
        .win_step(win_step), .chip_width(chip_width), .amplitude(amplitude),
        .pulse_width(pulse_width), .listen_width(listen_width),
        .waveform_mode(waveform_mode), .fc_khz(fc_khz), .params_valid(params_valid)
    );

    // ---------------- ping sequencer ----------------
    localparam [1:0] S_INIT = 2'd0, S_PREP = 2'd1, S_TX = 2'd2, S_LISTEN = 2'd3;
    reg [1:0]  state;
    reg [23:0] cnt;

    reg [31:0]        freq_l, wstep_l;
    reg signed [47:0] chirp_l;
    reg [39:0]        geo_l;
    reg [23:0]        chip_l, pw_l, listen_l;
    reg [7:0]         amp_l;
    reg [2:0]         mode_l, win_l;

    wire cnt_last_prep   = (cnt + 24'd1 >= LEAD);
    wire cnt_last_tx     = (cnt + 24'd1 >= pw_l);
    wire cnt_last_listen = (cnt + 24'd1 >= listen_l);

    task latch_params;
        begin
            freq_l <= freq_word;   chirp_l <= chirp_rate; geo_l   <= geo_gain;
            wstep_l <= win_step;   chip_l  <= chip_width; amp_l   <= amplitude;
            pw_l   <= pulse_width; listen_l <= listen_width;
            mode_l <= waveform_mode;
            // a full-length taper ruins Barker sidelobes (-22 dB -> -5 dB with Hann),
            // so phase-coded pings get a 5 % Tukey edge taper unless rect is chosen
            win_l  <= (waveform_mode == 3'd3 && win_sel != 3'd0) ? 3'd5 : win_sel;
        end
    endtask

    always @(posedge clk) begin
        if (rst_s) begin
            state <= S_INIT; cnt <= 24'd0;
            freq_l <= 32'd0; chirp_l <= 48'sd0; geo_l <= 40'd0; wstep_l <= 32'd0;
            chip_l <= 24'd1; amp_l <= 8'd0; pw_l <= 24'd1; listen_l <= 24'd1;
            mode_l <= 3'd0; win_l <= 3'd1;
        end else if (tick) begin
            case (state)
            S_INIT:   if (params_valid) begin latch_params; cnt <= 24'd0; state <= S_PREP; end
            S_PREP:   if (cnt_last_prep) begin cnt <= 24'd0; state <= S_TX; end
                      else cnt <= cnt + 24'd1;
            S_TX:     if (cnt_last_tx) begin cnt <= 24'd0; state <= S_LISTEN; end
                      else cnt <= cnt + 24'd1;
            S_LISTEN: if (cnt_last_listen) begin latch_params; cnt <= 24'd0; state <= S_PREP; end
                      else cnt <= cnt + 24'd1;
            endcase
        end
    end

    wire active = (state == S_TX);
    wire first  = active && (cnt == 24'd0);
    wire dds_on = active || ((state == S_LISTEN) && (cnt < FLUSH));
    wire dds_ce = tick && dds_on;

    always @(posedge clk) begin
        if (rst_s) begin
            tx_active <= 1'b0; amp_en <= 1'b0; pulse_start <= 1'b0;
        end else begin
            tx_active   <= active;
            amp_en      <= (state == S_PREP) || active ||
                           ((state == S_LISTEN) && (cnt < TAIL));
            pulse_start <= tick && first;
        end
    end

    assign active_mode      = mode_l;
    assign dbg_pulse_width  = pw_l;
    assign dbg_listen_width = listen_l;
    assign dbg_amplitude    = amp_l;

    // ---------------- DDS ----------------
    dds_waveform_gen #(
        .OUT_W(OUT_W), .SINE_FILE(SINE_FILE), .WIN_FILE(WIN_FILE)
    ) u_dds (
        .clk(clk), .rst(rst_s), .ce(dds_ce),
        .active(active), .first(first), .mode(mode_l),
        .freq_word(freq_l), .chirp_rate(chirp_l), .geo_gain(geo_l),
        .win_step(wstep_l), .chip_width(chip_l), .amplitude(amp_l), .win_sel(win_l),
        .sample_out(sample_out), .dac_data(dac_data), .sample_valid(sample_valid),
        .sample16(sample16), .dbg_freq(dbg_freq)
    );

endmodule
