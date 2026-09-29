// ============================================================================
// XADC reader: 4 potentiometers on VAUX channels + die temperature / VCCINT
// ----------------------------------------------------------------------------
// Uses the Artix-7's built-in 12-bit, 1 MS/s XADC in continuous-sequencer
// mode with 64-sample hardware averaging (kills pot/ADC noise for free).
// Standard DRP pattern: every EOC, read the result register whose address
// equals CHANNEL (VAUXn -> 0x10+n, temperature -> 0x00, VCCINT -> 0x01).
//
// !! XADC input range is 0 .. 1.0 V (unipolar). Wire each pot as
//      3.3 V --[ 24k ]--+-- pot 10k --+-- GND      wiper -> VAUXP, VAUXN -> GND
//    so the wiper swings 0 .. ~0.97 V. Put 10 nF from each VAUXP to GND
//    right at the header (charge reservoir for the XADC sampling cap).
// !! AUX_A/B/C/D must be VAUX channels that are actually routed to headers on
//    your board -- check the EDGE Artix-7 schematic, then set the matching
//    PACKAGE_PINs in the .xdc.
// ============================================================================
module xadc_pots #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer AUX_A  = 0,      // depth
    parameter integer AUX_B  = 1,      // turbidity
    parameter integer AUX_C  = 2,      // temperature
    parameter integer AUX_D  = 3       // salinity
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [3:0]  vaux_p,
    input  wire [3:0]  vaux_n,
    output reg  [11:0] pot_a,
    output reg  [11:0] pot_b,
    output reg  [11:0] pot_c,
    output reg  [11:0] pot_d,
    output reg  [11:0] die_temp,      // degC = code * 503.975 / 4096 - 273.15
    output reg  [11:0] vccint         // V    = code * 3 / 4096
);
    localparam integer CD = (CLK_HZ + 25_999_999) / 26_000_000;   // ADCCLK <= 26 MHz
    localparam [15:0] AUX_MASK = (16'd1 << AUX_A) | (16'd1 << AUX_B) | (16'd1 << AUX_C) | (16'd1 << AUX_D);

    wire [15:0] vp16, vn16;
    genvar i;
    generate for (i = 0; i < 16; i = i + 1) begin : g_aux
        assign vp16[i] = (i == AUX_A) ? vaux_p[0] : (i == AUX_B) ? vaux_p[1] :
                         (i == AUX_C) ? vaux_p[2] : (i == AUX_D) ? vaux_p[3] : 1'b0;
        assign vn16[i] = (i == AUX_A) ? vaux_n[0] : (i == AUX_B) ? vaux_n[1] :
                         (i == AUX_C) ? vaux_n[2] : (i == AUX_D) ? vaux_n[3] : 1'b0;
    end endgenerate

    wire [4:0]  channel;
    wire        eoc, drdy;
    wire [15:0] do_w;
    reg         den;
    reg  [6:0]  daddr;

    XADC #(
        .INIT_40(16'h2000),                  // 64-sample averaging
        .INIT_41(16'h2FFF),                  // continuous sequencer, alarms off, calibration on
        .INIT_42({CD[7:0], 8'h00}),          // DCLK divider
        .INIT_48(16'h0301),                  // sequence: calibration, temperature, VCCINT
        .INIT_49(AUX_MASK),                  // sequence: the three pots
        .INIT_4A(16'h0300),                  // average temperature, VCCINT
        .INIT_4B(AUX_MASK),                  // average the pots
        .SIM_DEVICE("7SERIES")
    ) u_xadc (
        .DCLK(clk), .RESET(rst),
        .DADDR(daddr), .DEN(den), .DWE(1'b0), .DI(16'h0000),
        .DO(do_w), .DRDY(drdy),
        .CONVST(1'b0), .CONVSTCLK(1'b0),
        .VAUXP(vp16), .VAUXN(vn16),
        .CHANNEL(channel), .EOC(eoc),
        .ALM(), .OT(), .EOS(), .BUSY(),
        .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED(), .MUXADDR()
    );

    reg [4:0] ch_q;
    always @(posedge clk) begin
        if (rst) begin
            den <= 1'b0; daddr <= 7'd0; ch_q <= 5'd0;
            pot_a <= 12'd0; pot_b <= 12'd0; pot_c <= 12'h800; pot_d <= 12'hDF0;
            die_temp <= 12'd0; vccint <= 12'd0;
        end else begin
            den <= 1'b0;
            if (eoc) begin
                den <= 1'b1; daddr <= {2'b00, channel}; ch_q <= channel;
            end
            if (drdy) begin
                if      (ch_q == 5'h10 + AUX_A) pot_a    <= do_w[15:4];
                else if (ch_q == 5'h10 + AUX_B) pot_b    <= do_w[15:4];
                else if (ch_q == 5'h10 + AUX_C) pot_c    <= do_w[15:4];
                else if (ch_q == 5'h10 + AUX_D) pot_d    <= do_w[15:4];
                else if (ch_q == 5'h00)         die_temp <= do_w[15:4];
                else if (ch_q == 5'h01)         vccint   <= do_w[15:4];
            end
        end
    end
endmodule
