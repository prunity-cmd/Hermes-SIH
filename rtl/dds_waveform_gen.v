// ============================================================================
// DDS waveform generator  (v3)
// ----------------------------------------------------------------------------
// phase accumulator -> 1024-pt quarter-wave sine ROM (BRAM) -> x full-length
// window (BRAM) x amplitude -> rounded, saturated DAC code.
//
//  * One clock domain. Every pipeline stage advances on `ce` (= sample tick
//    while the DDS is running), so outside a ping the whole datapath -- and
//    both BRAMs, via their enable pins -- sits still. `power_opt_design`
//    turns those enables into BRAM/flop clock gating.
//  * Frequency register is Q32.16: 16 fractional bits keep chirp rates exact
//    (a 100 kHz sweep over 2.5 ms at 10 MS/s is 26.8 LSB/tick -- without the
//    fraction it would truncate to 26 and lose 3 % of the bandwidth).
//  * Geometric sweep is a true exponential: f += f * g every tick, with
//    g = ln(f_end/f_start) / T supplied by param_controller, so the sweep
//    always ends exactly at f_end whatever the pulse length.
//    The f*g multiply is pipelined on clk (3 clocks) -> needs SAMPLE_DIV >= 4.
//  * Barker-13: the chip timer runs on a fixed grid; each phase flip is then
//    applied at the NEXT carrier zero crossing (phase MSB toggles), so the
//    output voltage never jumps -- no edge stress, no splatter.
//  * Window: full-length (Hann / Hamming / Blackman / Tukey / rect), indexed
//    by its own phase accumulator (step 2^32/T), so it always spans exactly
//    one pulse regardless of length.
//
// Modes: 0 CW, 1 LFM up, 2 LFM down, 3 Barker-13, 4 geometric sweep
// Pipeline latency: 4 ce from stage-0 update to dac_data.
// ============================================================================

module dds_waveform_gen #(
    parameter integer OUT_W     = 8,            // DAC bits (R2R ladder)
    parameter         SINE_FILE = "sine_quarter.mem",
    parameter         WIN_FILE  = "window.mem"
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 ce,             // sample-rate enable (gated outside pings)
    input  wire                 active,         // inside the transmit pulse
    input  wire                 first,          // first sample of the pulse
    input  wire [2:0]           mode,
    input  wire [31:0]          freq_word,      // start tuning word
    input  wire signed [47:0]   chirp_rate,     // Q32.16 per tick
    input  wire [39:0]          geo_gain,       // Q0.48 per tick
    input  wire [31:0]          win_step,
    input  wire [23:0]          chip_width,
    input  wire [7:0]           amplitude,
    input  wire [2:0]           win_sel,        // 0 rect 1 Hann 2 Hamming 3 Blackman 4 Tukey.25 5 Tukey.05

    output reg  signed [OUT_W-1:0] sample_out,
    output reg  [OUT_W-1:0]     dac_data,       // offset binary, mid-scale = 0 V
    output reg                  sample_valid,
    output reg  signed [15:0]   sample16,       // high-res copy for analysis / testbench
    output wire [31:0]          dbg_freq        // instantaneous tuning word
);

    localparam [2:0] M_CW = 3'd0, M_UP = 3'd1, M_DOWN = 3'd2, M_BARKER = 3'd3, M_GEO = 3'd4;
    localparam integer SH   = 32 - OUT_W;
    localparam integer SMAX = (1 << (OUT_W - 1)) - 1;
    localparam [12:0]  BARKER13 = 13'b1111100110101;   // chip 0 = MSB, 1 = +1

    // ---------------- ROMs (inferred BRAM) ----------------
    reg [14:0] sine_rom [0:1023];
    reg [15:0] win_rom  [0:4095];
    initial begin
        $readmemh(SINE_FILE, sine_rom);
        $readmemh(WIN_FILE,  win_rom);
    end

    // ---------------- stage 0: accumulators ----------------
    reg [31:0] phase_acc, wphase;
    reg [47:0] freq_cur;                        // Q32.16
    reg        active0, inv0, pend;
    reg [23:0] chip_cnt;
    reg [3:0]  chip_idx, bnd_cnt;

    assign dbg_freq = freq_cur[47:16];

    // geometric step = f * g, pipelined on clk (valid 3 clocks after f changes)
    reg [31:0] g_a;
    reg [39:0] g_b;
    reg [71:0] g_p;
    reg [47:0] geo_step;
    always @(posedge clk) begin
        g_a      <= freq_cur[47:16];
        g_b      <= geo_gain;
        g_p      <= g_a * g_b;                  // Q32 x Q0.48 = Q32.48
        geo_step <= {8'd0, g_p[71:32]};           // >>32 -> Q32.16
    end

    wire [47:0] delta     = (mode == M_GEO) ? geo_step : chirp_rate;
    wire [31:0] phase_nx  = phase_acc + freq_cur[47:16];
    wire        crossing  = phase_nx[31] ^ phase_acc[31];     // sine zero crossing
    wire        chip_exp  = (mode == M_BARKER) && (chip_cnt + 24'd1 >= chip_width)
                            && (bnd_cnt < 4'd12);
    wire        pend_now  = pend | chip_exp;
    wire [3:0]  idx_nx    = chip_idx + 4'd1;

    always @(posedge clk) begin
        if (rst) begin
            phase_acc <= 32'd0; wphase <= 32'd0; freq_cur <= 48'd0;
            active0 <= 1'b0; inv0 <= 1'b0; pend <= 1'b0;
            chip_cnt <= 24'd0; chip_idx <= 4'd0; bnd_cnt <= 4'd0;
        end else if (ce) begin
            active0 <= active;
            if (active && first) begin
                phase_acc <= 32'd0;
                wphase    <= 32'd0;
                freq_cur  <= {freq_word, 16'd0};
                inv0 <= 1'b0; pend <= 1'b0;
                chip_cnt <= 24'd0; chip_idx <= 4'd0; bnd_cnt <= 4'd0;
            end else if (active) begin
                phase_acc <= phase_nx;
                wphase    <= wphase + win_step;
                freq_cur  <= freq_cur + delta;
                if (mode == M_BARKER) begin
                    if (chip_exp) begin chip_cnt <= 24'd0; bnd_cnt <= bnd_cnt + 4'd1; end
                    else            chip_cnt <= chip_cnt + 24'd1;
                    if (pend_now && crossing) begin
                        chip_idx <= idx_nx;
                        inv0     <= ~BARKER13[4'd12 - idx_nx];
                        pend     <= 1'b0;
                    end else
                        pend     <= pend_now;
                end
            end
        end
    end

    // ---------------- stage 1: ROM reads ----------------
    wire [1:0] quad  = phase_acc[31:30];
    wire [9:0] kidx  = phase_acc[29:20];
    wire [9:0] k_eff = quad[0] ? ~kidx : kidx;
    wire [9:0] widx  = wphase[31:22];
    wire [8:0] hidx  = widx[9] ? ~widx[8:0] : widx[8:0];      // mirror 2nd half

    reg [14:0] mag1;
    reg [15:0] win1;
    reg        neg1, act1;
    always @(posedge clk) begin
        if (rst) begin
            mag1 <= 15'd0; win1 <= 16'd0; neg1 <= 1'b0; act1 <= 1'b0;
        end else if (ce) begin
            mag1 <= sine_rom[k_eff];
            win1 <= win_rom[{win_sel, hidx}];
            neg1 <= quad[1] ^ inv0;
            act1 <= active0;
        end
    end

    // ---------------- stage 2: sign + gain ----------------
    wire [23:0] gain_full = amplitude * win1;
    reg signed [15:0] s2;
    reg        [15:0] gain2;
    reg               act2;
    always @(posedge clk) begin
        if (rst) begin
            s2 <= 16'sd0; gain2 <= 16'd0; act2 <= 1'b0;
        end else if (ce) begin
            s2    <= neg1 ? -$signed({1'b0, mag1}) : $signed({1'b0, mag1});
            gain2 <= act1 ? gain_full[23:8] : 16'd0;
            act2  <= act1;
        end
    end

    // ---------------- stage 3: multiply (DSP48: 16 x 17) ----------------
    reg signed [32:0] prod3;
    reg               act3;
    always @(posedge clk) begin
        if (rst) begin
            prod3 <= 33'sd0; act3 <= 1'b0;
        end else if (ce) begin
            prod3 <= s2 * $signed({1'b0, gain2});
            act3  <= act2;
        end
    end

    // ---------------- stage 4: round, saturate, offset binary ----------------
    wire signed [33:0] r_out = ($signed({prod3[32], prod3}) + (34'sd1 <<< (SH - 1))) >>> SH;
    wire signed [33:0] r_16  = ($signed({prod3[32], prod3}) + (34'sd1 <<< 15)) >>> 16;
    wire signed [OUT_W-1:0] sat_out =
        (r_out >  SMAX) ?  SMAX[OUT_W-1:0] :
        (r_out < -SMAX) ? -SMAX[OUT_W-1:0] : r_out[OUT_W-1:0];

    always @(posedge clk) begin
        if (rst) begin
            sample_out   <= {OUT_W{1'b0}};
            dac_data     <= {1'b1, {(OUT_W-1){1'b0}}};
            sample_valid <= 1'b0;
            sample16     <= 16'sd0;
        end else if (ce) begin
            sample_out   <= sat_out;
            dac_data     <= {~sat_out[OUT_W-1], sat_out[OUT_W-2:0]};
            sample_valid <= act3;
            sample16     <= r_16[15:0];
        end
    end

endmodule
