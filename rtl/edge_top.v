// ============================================================================
// Board wrapper, EDGE Artix-7 (XC7A35T): adaptive sonar -> 8-bit R2R ladder
// ----------------------------------------------------------------------------
// !! CHECK CLK_HZ against the board: your .xdc create_clock period must match
//    (the deck says 50 MHz oscillator; the old wrapper assumed 100 MHz).
//    FS_HZ = 10 MS/s needs CLK_HZ/FS_HZ >= 4 and an integer ratio:
//    100 MHz -> /10, 50 MHz -> /5. Everything else retunes itself.
//
// Sensors: 4 pots via XADC (default) -- flip VIO probe_out6 to 1 to use the
// VIO sliders instead (bring-up fallback, no pots needed).
//
// VIO IP "vio_0" (regenerate with exactly these probes):
//   probe_out0  1 bit   mode_override_en   init 0
//   probe_out1  3 bits  mode_override      init 1  (0 CW 1 up 2 down 3 Barker 4 geo)
//   probe_out2  3 bits  win_sel            init 1  (0 rect 1 Hann 2 Hamming 3 Blackman 4 Tukey)
//   probe_out3  8 bits  vio depth          init 0x14
//   probe_out4  8 bits  vio turbidity      init 0x0A
//   probe_out5  8 bits  vio temperature    init 0x80
//   probe_out6  1 bit   use_vio_sensors    init 0
//   probe_out7  8 bits  vio salinity       init 0xDF  (0xDF = 35 ppt, 0x60 = 15, 0x00 = fresh)
//   probe_in0   5 bits  {amp_en, tx_active, active_mode[2:0]}
//   probe_in1  10 bits  fc_khz            (live centre frequency)
//   probe_in2  32 bits  {depth, turbidity, temperature, salinity} actually used
//   probe_in3  24 bits  pulse width, samples (/10 = us at 10 MS/s)
//   probe_in4   8 bits  amplitude
//   probe_in5  12 bits  XADC die temperature code
//
// Pins:  r2r[7:0] -> ladder (bit 7 = MSB), amp_en -> front-end power switch,
//        scope_trig -> scope external trigger (high for the whole pulse).
// ============================================================================
module edge_top #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer BTN_ACTIVE_HIGH = 1,
    parameter integer AUX_DEPTH       = 6,    // J13: C3 / C2
    parameter integer AUX_TURB        = 14,   // J13: B2 / A2
    parameter integer AUX_TEMP        = 7,    // J13: C1 / B1
    parameter integer AUX_SAL         = 15    // J13: E2 / D1
)(
    input  wire       clk,
    input  wire       btn_rst,
    input  wire [3:0] vaux_p,
    input  wire [3:0] vaux_n,
    output wire [3:0] led,
    output wire [7:0] r2r,
    output wire       amp_en,
    output wire       scope_trig
);
    wire rst_i = BTN_ACTIVE_HIGH ? btn_rst : ~btn_rst;

    // ---------------- sensors ----------------
    wire [11:0] pot_d, pot_t, pot_tp, pot_s, die_temp, vccint;
    xadc_pots #(.CLK_HZ(CLK_HZ), .AUX_A(AUX_DEPTH), .AUX_B(AUX_TURB),
                .AUX_C(AUX_TEMP), .AUX_D(AUX_SAL)) u_adc (
        .clk(clk), .rst(rst_i), .vaux_p(vaux_p), .vaux_n(vaux_n),
        .pot_a(pot_d), .pot_b(pot_t), .pot_c(pot_tp), .pot_d(pot_s),
        .die_temp(die_temp), .vccint(vccint)
    );

    wire       ov_en, use_vio;
    wire [2:0] ov_mode, win_sel;
    wire [7:0] vio_d, vio_t, vio_tp, vio_s;

    wire [7:0] depth = use_vio ? vio_d  : pot_d[11:4];
    wire [7:0] turb  = use_vio ? vio_t  : pot_t[11:4];
    wire [7:0] temp  = use_vio ? vio_tp : pot_tp[11:4];
    wire [7:0] sal   = use_vio ? vio_s  : pot_s[11:4];

    // ---------------- transmitter ----------------
    wire [7:0]  dac_data, amp_l;
    wire [2:0]  active_mode;
    wire [9:0]  fc_khz;
    wire [23:0] pw_l;
    wire        tx_active, amp_en_i;

    adaptive_sonar_top #(
        .CLK_HZ(CLK_HZ), .FS_HZ(10_000_000), .OUT_W(8)
    ) u_top (
        .clk(clk), .rst(rst_i),
        .depth(depth), .turbidity(turb), .temperature(temp), .salinity(sal),
        .mode_override_en(ov_en), .mode_override(ov_mode), .win_sel(win_sel),
        .dac_data(dac_data), .sample_out(), .sample16(), .sample_valid(),
        .sample_tick(), .waveform_mode(), .active_mode(active_mode), .fc_khz(fc_khz),
        .tx_active(tx_active), .amp_en(amp_en_i), .pulse_start(),
        .dbg_freq(), .dbg_pulse_width(pw_l), .dbg_listen_width(), .dbg_amplitude(amp_l)
    );

    // ---------------- outputs: all registered in the IOB ----------------
    // Same-edge IOB flops keep bit-to-bit skew to ~100 ps, which is what
    // keeps R2R code-transition glitches small.
    (* IOB = "TRUE" *) reg [7:0] r2r_q  = 8'h80;
    (* IOB = "TRUE" *) reg       amp_q  = 1'b0;
    (* IOB = "TRUE" *) reg       trig_q = 1'b0;
    always @(posedge clk) begin
        r2r_q  <= dac_data;
        amp_q  <= amp_en_i;
        trig_q <= tx_active;
    end
    assign r2r        = r2r_q;
    assign amp_en     = amp_q;
    assign scope_trig = trig_q;
    assign led        = {tx_active, active_mode};

    vio_0 u_vio (
        .clk(clk),
        .probe_in0({amp_en_i, tx_active, active_mode}),
        .probe_in1(fc_khz),
        .probe_in2({depth, turb, temp, sal}),
        .probe_in3(pw_l),
        .probe_in4(amp_l),
        .probe_in5(die_temp),
        .probe_out0(ov_en),
        .probe_out1(ov_mode),
        .probe_out2(win_sel),
        .probe_out3(vio_d),
        .probe_out4(vio_t),
        .probe_out5(vio_tp),
        .probe_out6(use_vio),
        .probe_out7(vio_s)
    );
endmodule
