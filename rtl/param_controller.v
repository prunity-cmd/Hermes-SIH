// ============================================================================
// Adaptive Parameter Controller  (v3: continuous, physics-driven)
// ----------------------------------------------------------------------------
// Inputs (8-bit, from pots via XADC, or VIO sliders in fallback mode):
//   depth       0..255  -> required range (R_MIN_M .. R_MAX_M)
//   turbidity   0..255  -> suspended sediment (scattering loss)
//   temperature 0..255  -> 0..35 degC (sound-speed correction of listen window)
//   salinity    0..255  -> 0..40 ppt  (absorption + sound speed)
//
// Continuous mapping -- every dial position gives a different waveform:
//   d_eff = depth * k(S)                   absorption-weighted path length:
//                                          k = alpha(S)/alpha(35 ppt) at 300 kHz
//                                          (Francois-Garrison: MgSO4 relaxation
//                                          scales with S; fresh water ~0.27x)
//   s_f = max(d_eff, turbidity)            "loss stress": lossy path OR mud
//   s_e = max(depth, 3/4 * turbidity)      "energy stress" (raw depth: spherical
//                                          spreading, not absorption, dominates
//                                          the energy budget at <= 150 m)
//   centre freq   fc = F_MAX - (F_MAX-F_MIN) * s_f/256
//                 (absorption rises steeply with f: go low when the path is
//                  long or muddy, high when clear -> best resolution)
//   bandwidth     B  = fc * 51/256         (~20 % constant fractional BW)
//   pulse length  T  = PW_MIN + (PW_MAX-PW_MIN) * s_e/256   (energy)
//   amplitude     A  = A_MIN  + (255-A_MIN)     * s_e/256   (power)
//   listen        L  = 2 R / c(T,S)        R from depth, c from Medwin (ROMs)
//
// Modulation (Schmitt-trigger flags, or manual override for demos):
//   deep && turbid -> 3 Barker-13 phase-coded (worst case)
//   turbid         -> 4 geometric sweep       (muddy estuary)
//   otherwise      -> 1 LFM up-chirp          (clear reef / deep open water)
//   override       -> 0 CW, 1 LFM up, 2 LFM down, 3 Barker, 4 geo
//
// Rates that need "/ pulse_length" come from one small iterative divider,
// run three times in sequence (~160 clocks). The whole set is then COMMITTED
// in a single clock, so the top level can never latch a half-updated set.
//
// Output units:
//   freq_word   32-bit DDS tuning word of the pulse START frequency
//   chirp_rate  signed Q32.16 tuning-word step per tick      (LFM)
//   geo_gain    Q0.48 exponential growth per tick             (geo sweep)
//   win_step    window-phase step per tick = 2^32 / pulse_width
//   chip_width  ticks per Barker chip = pulse_width / 13
// ============================================================================

module param_controller #(
    parameter integer FS_HZ      = 10_000_000,
    // whole swept band stays inside 100-500 kHz
    parameter integer F_MIN_HZ   = 112_000,     // band 101-123 kHz
    parameter integer F_MAX_HZ   = 450_000,     // band 405-495 kHz
    parameter integer BW_FRAC_Q8 = 51,          // B = fc * 51/256
    parameter [47:0]  LN_R_Q48   = 48'd56261664523027,  // from gen_tables.py
    parameter integer PW_MIN_US  = 1000,
    parameter integer PW_MAX_US  = 8000,
    parameter integer A_MIN      = 128,         // 50 % amplitude floor
    parameter integer R_MIN_M    = 10,
    parameter integer R_MAX_M    = 150,
    parameter integer DEEP_TH    = 160,
    parameter integer TURB_TH    = 150,
    parameter integer HYST       = 6,
    parameter integer DEADBAND   = 2,           // ignore +/-1 LSB ADC flicker
    parameter         SS_FILE    = "soundspeed.mem",
    parameter         SAL_FILE   = "salinity.mem"
)(
    input  wire        clk,
    input  wire        rst,                     // synchronous, active high
    input  wire [7:0]  depth,
    input  wire [7:0]  turbidity,
    input  wire [7:0]  temperature,
    input  wire [7:0]  salinity,
    input  wire        mode_override_en,
    input  wire [2:0]  mode_override,

    output reg  [31:0]        freq_word,
    output reg  signed [47:0] chirp_rate,
    output reg  [39:0]        geo_gain,
    output reg  [31:0]        win_step,
    output reg  [23:0]        chip_width,
    output reg  [7:0]         amplitude,
    output reg  [23:0]        pulse_width,
    output reg  [23:0]        listen_width,
    output reg  [2:0]         waveform_mode,
    output reg  [9:0]         fc_khz,
    output reg                params_valid
);

    // ---------------- elaboration-time constants ----------------
    localparam [63:0] TWO32   = 64'd1 << 32;
    localparam [63:0] W_MAX   = (F_MAX_HZ * TWO32) / FS_HZ;
    localparam [63:0] W_MIN   = (F_MIN_HZ * TWO32) / FS_HZ;
    localparam [63:0] W_SPAN  = W_MAX - W_MIN;
    localparam [63:0] PW_MIN  = (PW_MIN_US * 64'd1 * FS_HZ) / 1_000_000;
    localparam [63:0] PW_SPAN = ((PW_MAX_US - PW_MIN_US) * 64'd1 * FS_HZ) / 1_000_000;
    localparam [63:0] RT_MIN  = (2 * R_MIN_M * 64'd1 * FS_HZ) / 1500;   // ticks @ 1500 m/s
    localparam [63:0] RT_SPAN = (2 * (R_MAX_M - R_MIN_M) * 64'd1 * FS_HZ) / 1500;
    localparam [63:0] KHZ_K   = FS_HZ / 1000;

    localparam [2:0] M_CW = 3'd0, M_UP = 3'd1, M_DOWN = 3'd2, M_BARKER = 3'd3, M_GEO = 3'd4;

    reg [15:0] ss_rom [0:255];                  // Q1.15 of 1500 / c(T)
    initial $readmemh(SS_FILE, ss_rom);
    reg [31:0] sal_rom [0:255];                 // {7'b0, k Q1.8, spd Q1.15}
    initial $readmemh(SAL_FILE, sal_rom);

    function [7:0] db;                          // deadband
        input [7:0] cur, in;
        begin
            db = (((in > cur) ? (in - cur) : (cur - in)) >= DEADBAND) ? in : cur;
        end
    endfunction

    // ---------------- state ----------------
    localparam [3:0] S_SAMPLE = 0, S_C1 = 1, S_C2 = 2, S_C3 = 3, S_C4 = 4,
                     S_DIV_LFM = 5, S_DIV_GEO = 6, S_DIV_WIN = 7, S_COMMIT = 8,
                     S_C1B = 9, S_C3B = 10;
    reg [3:0]  st;
    reg [7:0]  d_u, t_u, tp_u, sl_u;
    reg [31:0] sal_q;
    reg        deep, turbid;
    reg [7:0]  s_f, s_e;
    reg [31:0] fc_w, bw_w, start_w;
    reg [23:0] pw_n, rt_n, lst_n;
    reg [39:0] lst_a;                           // rt_n * ss_q, pipelined (timing)
    reg [7:0]  amp_n;
    reg [2:0]  mode_n;
    reg [47:0] rate_q, gain_q, wstep_q;
    reg [15:0] ss_q;

    // ---------------- shared iterative divider (restoring, 48 steps) ----------------
    reg        div_go, div_busy, div_started;
    reg [47:0] div_num, div_num_in, div_quot;
    reg [23:0] div_den;
    reg [24:0] div_rem;
    reg [5:0]  div_i;
    wire [24:0] div_trial = {div_rem[23:0], div_num[47]};
    wire        div_done  = div_started && !div_busy && !div_go;

    always @(posedge clk) begin
        if (rst) begin
            div_busy <= 1'b0; div_i <= 6'd0; div_rem <= 25'd0;
            div_quot <= 48'd0; div_num <= 48'd0;
        end else if (div_go) begin
            div_busy <= 1'b1; div_i <= 6'd48; div_rem <= 25'd0;
            div_quot <= 48'd0; div_num <= div_num_in;
        end else if (div_busy) begin
            if (div_trial >= {1'b0, div_den}) begin
                div_rem  <= div_trial - {1'b0, div_den};
                div_quot <= {div_quot[46:0], 1'b1};
            end else begin
                div_rem  <= div_trial;
                div_quot <= {div_quot[46:0], 1'b0};
            end
            div_num <= {div_num[46:0], 1'b0};
            div_i   <= div_i - 6'd1;
            if (div_i == 6'd1) div_busy <= 1'b0;
        end
    end

    // wide intermediates (explicit 64-bit so nothing overflows)
    wire [63:0] fc_calc  = W_MAX - ((W_SPAN * s_f) >> 8);
    wire [63:0] pw_calc  = PW_MIN + ((PW_SPAN * s_e) >> 8);
    wire [63:0] rt_calc  = RT_MIN + ((RT_SPAN * d_u) >> 8);
    wire [63:0] bw_calc  = ({32'd0, fc_w} * BW_FRAC_Q8) >> 8;
    // listen = rt * ss(T) * spd(S) >> 30, split over two clocks so each
    // multiply fits in one DSP stage at 100 MHz
    wire [39:0] lst_a_calc = rt_n * ss_q;                                  // 24 x 16
    wire [63:0] lst_calc   = ({24'd0, lst_a} * {48'd0, sal_q[15:0]}) >> 30; // 40 x 16
    wire [16:0] d_k      = d_u * sal_q[24:16];                      // depth x k(S), Q1.8
    wire [7:0]  d_eff    = (d_k[16:8] > 9'd255) ? 8'd255 : d_k[15:8];
    wire [63:0] chip_calc= ({40'd0, pw_n} * 64'd5041) >> 16;           // / 13
    wire [63:0] khz_calc = ({32'd0, fc_w} * KHZ_K) >> 32;
    wire [7:0]  t34      = t_u - (t_u >> 2);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_SAMPLE; div_go <= 1'b0; div_started <= 1'b0;
            d_u <= 8'd0; t_u <= 8'd0; tp_u <= 8'd128; sl_u <= 8'd223; sal_q <= 32'h0100_8000;
            deep <= 1'b0; turbid <= 1'b0;
            s_f <= 8'd0; s_e <= 8'd0; ss_q <= 16'h8000;
            fc_w <= 32'd0; bw_w <= 32'd0; start_w <= 32'd0;
            pw_n <= 24'd1; rt_n <= 24'd1; lst_n <= 24'd1; lst_a <= 40'd0; amp_n <= 8'd0; mode_n <= M_UP;
            rate_q <= 48'd0; gain_q <= 48'd0; wstep_q <= 48'd0; div_num_in <= 48'd0; div_den <= 24'd1;
            params_valid <= 1'b0;
            freq_word <= W_MAX[31:0]; chirp_rate <= 48'sd0; geo_gain <= 40'd0;
            win_step <= 32'd0; chip_width <= 24'd1; amplitude <= 8'd0;
            pulse_width <= 24'd1; listen_width <= 24'd1; waveform_mode <= M_CW;
            fc_khz <= 10'd0;
        end else begin
            div_go <= 1'b0;
            case (st)
            S_SAMPLE: begin
                d_u  <= db(d_u,  depth);
                t_u  <= db(t_u,  turbidity);
                tp_u <= db(tp_u, temperature);
                sl_u <= db(sl_u, salinity);
                st   <= S_C1;
            end
            S_C1: begin
                if      (d_u > DEEP_TH + HYST) deep   <= 1'b1;
                else if (d_u < DEEP_TH - HYST) deep   <= 1'b0;
                if      (t_u > TURB_TH + HYST) turbid <= 1'b1;
                else if (t_u < TURB_TH - HYST) turbid <= 1'b0;
                s_e  <= (d_u > t34) ? d_u : t34;
                ss_q <= ss_rom[tp_u];
                sal_q <= sal_rom[sl_u];
                st   <= S_C1B;
            end
            S_C1B: begin
                s_f  <= (d_eff > t_u) ? d_eff : t_u;
                st   <= S_C2;
            end
            S_C2: begin
                fc_w  <= fc_calc[31:0];
                pw_n  <= pw_calc[23:0];
                amp_n <= A_MIN + (((255 - A_MIN) * s_e) >> 8);
                rt_n  <= rt_calc[23:0];
                if      (mode_override_en) mode_n <= (mode_override > M_GEO) ? M_UP : mode_override;
                else if (deep && turbid)   mode_n <= M_BARKER;
                else if (turbid)           mode_n <= M_GEO;
                else                       mode_n <= M_UP;
                st <= S_C3;
            end
            S_C3: begin
                bw_w  <= bw_calc[31:0];
                lst_a <= lst_a_calc;
                st    <= S_C3B;
            end
            S_C3B: begin
                lst_n <= lst_calc[23:0];
                st    <= S_C4;
            end
            S_C4: begin
                case (mode_n)
                    M_UP, M_GEO: start_w <= fc_w - (bw_w >> 1);
                    M_DOWN:      start_w <= fc_w + (bw_w >> 1);
                    default:     start_w <= fc_w;
                endcase
                div_num_in  <= {bw_w, 16'd0};      // LFM: (B<<16)/T -> Q32.16
                div_den     <= pw_n;
                div_go      <= 1'b1;
                div_started <= 1'b1;
                st <= S_DIV_LFM;
            end
            S_DIV_LFM: if (div_done) begin
                rate_q     <= div_quot;
                div_num_in <= LN_R_Q48;            // geo: ln(r)/T -> Q0.48
                div_go     <= 1'b1;
                st <= S_DIV_GEO;
            end
            S_DIV_GEO: if (div_done) begin
                gain_q     <= div_quot;
                div_num_in <= 48'h1_0000_0000;     // window: 2^32/T
                div_go     <= 1'b1;
                st <= S_DIV_WIN;
            end
            S_DIV_WIN: if (div_done) begin
                wstep_q       <= div_quot;
                div_started   <= 1'b0;
                st <= S_COMMIT;
            end
            S_COMMIT: begin
                freq_word     <= start_w;
                chirp_rate    <= (mode_n == M_UP)   ?  $signed(rate_q) :
                                 (mode_n == M_DOWN) ? -$signed(rate_q) : 48'sd0;
                geo_gain      <= (mode_n == M_GEO) ? gain_q[39:0] : 40'd0;
                win_step      <= wstep_q[31:0];
                chip_width    <= chip_calc[23:0];
                amplitude     <= amp_n;
                pulse_width   <= pw_n;
                listen_width  <= lst_n;
                waveform_mode <= mode_n;
                fc_khz        <= khz_calc[9:0];
                params_valid  <= 1'b1;
                st <= S_SAMPLE;
            end
            default: st <= S_SAMPLE;
            endcase
        end
    end

endmodule
