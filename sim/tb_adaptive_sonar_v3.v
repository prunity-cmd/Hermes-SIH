// ============================================================================
// Self-checking, continuous multi-scenario testbench for adaptive sonar v3.
// Runs the PS scenarios back to back and, for every checked ping, verifies:
//   * frequency at the start and end of the pulse (zero-crossing count,
//     independent of the DUT's own frequency register)
//   * pulse length (samples) and amplitude
//   * window edges: output starts and ends near 0 V (Hann/Blackman/Tukey)
//   * Barker-13: no voltage jumps at phase flips (max sample-to-sample step)
//   * amp_en rises >= AMP_LEAD before the first sample, DAC sits at
//     mid-scale while listening
// Also writes tx_samples.csv (ping, scenario, mode, n, sample16) for
// analyze_tx.py (spectrogram + FFT + pulse-compression sidelobes).
// Scaled for simulation speed: pulses 0.2-0.8 ms, range 1-5 m. Frequencies
// and sample rate are the REAL ones (100-500 kHz at 10 MS/s).
// ============================================================================
`timescale 1ns/1ps
// .mem location: Vivado finds them by name (leave MEM_DIR empty);
// Icarus from sim/:  iverilog -DMEM_DIR=\"../mem/\" ...
`ifndef MEM_DIR
`define MEM_DIR ""
`endif
module tb_adaptive_sonar_v3;

    localparam integer CLK_HZ = 40_000_000;   // SAMPLE_DIV = 4, the minimum
    localparam integer FS     = 10_000_000;
    localparam real    FSR    = 10.0e6;

    reg clk = 0, rst = 1;
    always #12.5 clk = ~clk;

    reg [7:0] depth = 20, turb = 10, temp = 128, sal = 223;
    reg       ov_en = 0;
    reg [2:0] ov_mode = 0, win_sel = 1;

    wire [7:0]  dac_data;
    wire signed [7:0]  sample_out;
    wire signed [15:0] sample16;
    wire sample_valid, sample_tick, tx_active, amp_en, pulse_start;
    wire [2:0] waveform_mode, active_mode;
    wire [9:0] fc_khz;
    wire [31:0] dbg_freq;
    wire [23:0] dbg_pw, dbg_lw;
    wire [7:0]  dbg_amp;

    adaptive_sonar_top #(
        .CLK_HZ(CLK_HZ), .FS_HZ(FS), .OUT_W(8),
        .PW_MIN_US(200), .PW_MAX_US(800), .R_MIN_M(1), .R_MAX_M(5),
        .AMP_LEAD_US(100), .AMP_TAIL_US(20),
        .SINE_FILE({`MEM_DIR, "sine_quarter.mem"}), .WIN_FILE({`MEM_DIR, "window.mem"}),
        .SS_FILE({`MEM_DIR, "soundspeed.mem"}), .SAL_FILE({`MEM_DIR, "salinity.mem"})
    ) dut (
        .clk(clk), .rst(rst),
        .depth(depth), .turbidity(turb), .temperature(temp), .salinity(sal),
        .mode_override_en(ov_en), .mode_override(ov_mode), .win_sel(win_sel),
        .dac_data(dac_data), .sample_out(sample_out), .sample16(sample16),
        .sample_valid(sample_valid), .sample_tick(sample_tick),
        .waveform_mode(waveform_mode), .active_mode(active_mode), .fc_khz(fc_khz),
        .tx_active(tx_active), .amp_en(amp_en), .pulse_start(pulse_start),
        .dbg_freq(dbg_freq), .dbg_pulse_width(dbg_pw),
        .dbg_listen_width(dbg_lw), .dbg_amplitude(dbg_amp)
    );

    // ---------------- capture ----------------
    integer buf_s [0:9999];
    integer n = 0, ping = 0, fails = 0, checks = 0, scen = 0;
    integer csv;
    reg     in_pulse = 0, check_this = 0;
    integer amp_rise_t = 0, lead_ticks = 0, tick_count = 0;
    reg     amp_q = 0;
    integer listen_bad = 0;
    reg [8*24-1:0] scen_name;

    // same packed table the RTL uses, for the integer-exact d_eff
    reg [31:0] sal_tab [0:255];
    initial $readmemh({`MEM_DIR, "salinity.mem"}, sal_tab);

    function real medwin;             // Medwin 1975, z = 0
        input real T, S;
        medwin = 1449.2 + 4.6*T - 0.055*T*T + 0.00029*T*T*T + (1.34 - 0.01*T)*(S - 35.0);
    endfunction

    // expected (set per scenario)
    real e_f0, e_f1;           // freq at start / end of pulse
    integer e_n, e_amp;
    real    e_listen;
    reg e_edge_zero, e_barker;

    always @(posedge clk) if (sample_tick) tick_count <= tick_count + 1;

    always @(posedge clk) begin
        amp_q <= amp_en;
        if (amp_en && !amp_q) amp_rise_t <= tick_count;
        if (sample_tick) begin
            if (sample_valid) begin
                if (!in_pulse) begin
                    in_pulse <= 1; n = 0;
                    lead_ticks = tick_count - amp_rise_t;
                end
                buf_s[n] = sample16;
                $fwrite(csv, "%0d,%0d,%0d,%0d,%0d\n", ping, scen, active_mode, n, sample16);
                n = n + 1;
            end else if (in_pulse) begin
                in_pulse <= 0;
                if (check_this) analyse();
                ping = ping + 1;
            end else if (!amp_en && dac_data != 8'h80 && !rst) begin
                listen_bad = listen_bad + 1;
            end
        end
    end

    // ---------------- analysis helpers ----------------
    function real seg_freq;           // zero-crossing frequency over [a, b)
        input integer a, b;
        integer i, zc, first_zc, last_zc;
        begin
            zc = 0; first_zc = -1; last_zc = -1;
            for (i = a + 1; i < b; i = i + 1)
                if ((buf_s[i-1] < 0 && buf_s[i] >= 0) || (buf_s[i-1] >= 0 && buf_s[i] < 0)) begin
                    if (first_zc < 0) first_zc = i;
                    last_zc = i; zc = zc + 1;
                end
            if (zc < 3) seg_freq = 0.0;
            else seg_freq = (zc - 1) / 2.0 / ((last_zc - first_zc) / FSR);
        end
    endfunction

    task check;
        input ok;
        input [8*48-1:0] what;
        begin
            checks = checks + 1;
            if (!ok) begin fails = fails + 1; $display("    FAIL: %0s", what); end
        end
    endtask

    task analyse;
        integer i, pk, mx_step, st, a, b, seg;
        real f0, f1, tol, fexp0, fexp1, step_lim;
        begin
            pk = 0; mx_step = 0;
            for (i = 0; i < n; i = i + 1) begin
                if (buf_s[i] > pk) pk = buf_s[i];
                if (-buf_s[i] > pk) pk = -buf_s[i];
                if (i > 0) begin
                    st = buf_s[i] - buf_s[i-1]; if (st < 0) st = -st;
                    if (st > mx_step) mx_step = st;
                end
            end
            seg = n / 5;
            // measure in the middle 60 % of the first / last fifth-ish windows
            // (away from the window taper so crossings are clean)
            f0 = seg_freq(n/10, n/10 + seg);
            f1 = seg_freq(n - n/10 - seg, n - n/10);
            // expected mean freq over those segments
            fexp0 = e_f0 + (e_f1 - e_f0) * ((n/10 + seg/2.0) / n);
            fexp1 = e_f0 + (e_f1 - e_f0) * ((n - n/10 - seg/2.0) / n);
            if (e_barker) begin fexp0 = e_f0; fexp1 = e_f0; end
            tol = e_barker ? 0.06 : 0.03;

            $display("  ping %0d [%0s] mode=%0d  n=%0d (exp %0d)  amp=%0d  peak=%0d",
                     ping, scen_name, active_mode, n, e_n, dbg_amp, pk);
            $display("    f_start %.1f kHz (exp %.1f)  f_end %.1f kHz (exp %.1f)  lead=%0d ticks  max_step=%0d",
                     f0/1e3, fexp0/1e3, f1/1e3, fexp1/1e3, lead_ticks, mx_step);

            check(n >= e_n - 2 && n <= e_n + 2, "pulse length");
            check(dbg_amp == e_amp, "amplitude");
            check(f0 > fexp0*(1-tol) && f0 < fexp0*(1+tol), "start frequency");
            check(f1 > fexp1*(1-tol) && f1 < fexp1*(1+tol), "end frequency");
            check(lead_ticks >= 1000, "amp_en lead >= 100 us");
            if (e_edge_zero) begin
                check(buf_s[0] < 400 && buf_s[0] > -400, "window starts at ~0 V");
                check(buf_s[n-1] < 400 && buf_s[n-1] > -400, "window ends at ~0 V");
            end
            // largest legit step for a sine of this peak at the top freq,
            // +10 % margin; a Barker flip mid-cycle would be ~2x this
            step_lim = 1.10 * 2.0 * 3.14159265 * ((e_f0 > e_f1) ? e_f0 : e_f1) / FSR * pk + 2;
            check(mx_step <= step_lim, "no voltage jumps (max sample step)");
        end
    endtask

    // ---------------- scenario driver ----------------
    // expected values computed from the controller's formulas
    task expect_auto;
        input integer d, t, tp, sl;
        input integer mode;         // expected waveform mode
        input integer wsel;
        real fc, bw, se;
        integer sf, s_e, t34, deff, rt;
        begin
            deff = (d * sal_tab[sl][24:16]) >> 8; if (deff > 255) deff = 255;
            sf  = (deff > t) ? deff : t;
            rt  = 13333 + (53333 * d) / 256;
            e_listen = rt * 1500.0 / medwin(tp * 35.0 / 255.0, sl * 40.0 / 255.0);
            t34 = t - (t >> 2);
            s_e = (d > t34) ? d : t34;
            fc  = 450.0e3 - (450.0e3 - 112.0e3) * sf / 256.0;
            bw  = fc * 51.0 / 256.0;
            e_n   = 2000 + (6000 * s_e) / 256;
            e_amp = 128 + ((127 * s_e) >> 8);
            e_barker = (mode == 3);
            e_edge_zero = (wsel == 1 || wsel == 3 || wsel == 4 || (mode == 3 && wsel != 0));
            case (mode)
                1, 4: begin e_f0 = fc - bw/2; e_f1 = fc + bw/2; end
                2:    begin e_f0 = fc + bw/2; e_f1 = fc - bw/2; end
                default: begin e_f0 = fc; e_f1 = fc; end
            endcase
        end
    endtask

    task run_scenario;
        input [8*24-1:0] name;
        input integer d, t, tp, sl, ov, ovm, ws, emode;
        begin
            scen = scen + 1; scen_name = name;
            depth = d; turb = t; temp = tp; sal = sl; ov_en = ov; ov_mode = ovm; win_sel = ws;
            expect_auto(d, t, tp, sl, emode, ws);
            $display("\n== Scenario %0d: %0s  (depth=%0d turb=%0d temp=%0d sal=%0d = %.1f ppt)",
                     scen, name, d, t, tp, sl, sl * 40.0 / 255.0);
            // the next ping may still carry old params -> skip it, check the one after
            check_this = 0;
            @(posedge pulse_start); @(negedge tx_active);
            check_this = 1;
            @(posedge pulse_start); #10;
            $display("  controller: mode=%0d fc=%0d kHz  listen=%0d ticks (%.1f m @1500 m/s)",
                     waveform_mode, fc_khz, dbg_lw, dbg_lw / FSR * 1500.0 / 2.0);
            $display("  listen %0d ticks, exact 2R/c(T,S) %.0f ticks (%.2f %%)",
                     dbg_lw, e_listen, 100.0 * (dbg_lw - e_listen) / e_listen);
            check(dbg_lw > e_listen * 0.995 && dbg_lw < e_listen * 1.005, "listen window = 2R/c(T,S)");
            check(active_mode == emode, "waveform mode");
            @(negedge tx_active);
            repeat (20) @(posedge sample_tick);
            check_this = 0;
        end
    endtask

    initial begin
        csv = $fopen("tx_samples.csv", "w");
        $fwrite(csv, "ping,scenario,mode,n,sample\n");
        repeat (10) @(posedge clk); rst = 0;

        // salinity code: 223 = 35 ppt (sea), 96 = 15 ppt (brackish), 0 = fresh
        //            name                    depth turb temp sal  ov ovm win exp_mode
        run_scenario("Clear shallow reef",      20,   10, 180, 223, 0, 0, 1,  1);
        run_scenario("Muddy estuary",           40,  210, 150,  96, 0, 0, 1,  4);
        run_scenario("Deep open water",        240,   20,  60, 223, 0, 0, 1,  1);
        run_scenario("Deep fresh water",       240,   20,  60,   0, 0, 0, 1,  1);
        run_scenario("Deep + muddy (worst)",   230,  220,  60, 223, 0, 0, 1,  3);
        run_scenario("Override CW, Blackman",   20,   10, 180, 223, 1, 0, 3,  0);
        run_scenario("Override down, Tukey",    20,   10, 180, 223, 1, 2, 4,  2);
        run_scenario("Override Barker, reef",   20,   10, 180, 223, 1, 3, 1,  3);
        run_scenario("Reef again (rect A/B)",   20,   10, 180, 223, 0, 0, 0,  1);

        $display("\nListen-window DAC samples off mid-scale: %0d", listen_bad);
        check(listen_bad == 0, "DAC at mid-scale while listening");
        $display("\n==== %0d checks, %0d failures -> %0s ====", checks, fails,
                 fails == 0 ? "ALL PASS" : "FAILED");
        $fclose(csv);
        $finish;
    end

    initial begin #400_000_000; $display("TIMEOUT"); $finish; end
endmodule
