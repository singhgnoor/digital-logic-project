`timescale 1ns/1ps

/*
seed_linear -> linear / piecewise linear seed for x0 = 1/d   (replaces the LUT when SEED_MODE = 1 or 2)
x0 = c0 - c1*d,  best (minimax) line on [a, b] for the relative error |d*x0 - 1|:
    c1 = 2 / (ab + (a+b)^2/4)      c0 = c1*(a+b)      worst error = (b-a)^2 / ((a+b)^2 + 4ab)
PWL = 0 : one line on [0.5, 1)  -> x0 = 48/17 - 32/17*d, error 1/17   (4.09 bits, same as the 8 entry LUT)
PWL = 1 : 8 lines, segment = d[29:27] = mb[22:20] (same index as the LUT), error ~2^-9.17
          -> 9.15 bits after the 16 bit truncation (checked on all 2^23 mantissas in the model), so 2 NR iterations are enough

Q formats
    d   : Q1.31, only d[30:15] is used (Q0.16, d[31] is always 0, d[30] is the hidden 1)
    c1  : Q2.14 (16 bit, max 3.55)
    c1*d: Q2.14 * Q0.16 = Q2.30, straight from vedic_16x16, no shift needed (value < 2)
    c0  : Q2.30 (max 3.77, fits below 4)
    x0  : c0 - c1*d through sub32, Q2.30
combinational (gate level multiplier + CLA subtractor), so it needs no extra FSM cycle
*/

module seed_linear (
    input  wire [31:0] d,  // Q1.31 divisor, in [0.5, 1)
    output wire [31:0] x0  // Q2.30 seed
);
    parameter PWL = 1; // 0 -> single line, 1 -> 8 segment piecewise linear

    reg [31:0] c0_t; // Q2.30, 8 segment table
    reg [15:0] c1_t; // Q2.14

    always @(*) begin
        case (d[29:27])
            3'd0: begin c0_t = 32'hF15BD72F; c1_t = 16'hE329; end  // [0.5000, 0.5625)  3.77123 - 3.54938 d
            3'd1: begin c0_t = 32'hD7E0C127; c1_t = 16'hB5CB; end  // [0.5625, 0.6250)  3.37309 - 2.84052 d
            3'd2: begin c0_t = 32'hC344DE01; c1_t = 16'h94C7; end  // [0.6250, 0.6875)  3.05108 - 2.32465 d
            3'd3: begin c0_t = 32'hB2416483; c1_t = 16'h7C01; end  // [0.6875, 0.7500)  2.78524 - 1.93756 d
            3'd4: begin c0_t = 32'hA3F89F0D; c1_t = 16'h68F1; end  // [0.7500, 0.8125)  2.56205 - 1.63971 d
            3'd5: begin c0_t = 32'h97CECD93; c1_t = 16'h59F6; end  // [0.8125, 0.8750)  2.37200 - 1.40564 d
            3'd6: begin c0_t = 32'h8D534D84; c1_t = 16'h4DF9; end  // [0.8750, 0.9375)  2.20821 - 1.21832 d
            3'd7: begin c0_t = 32'h8432A3EA; c1_t = 16'h443B; end  // [0.9375, 1.0000)  2.06559 - 1.06610 d
        endcase
    end

    wire [31:0] c0 = (PWL == 0) ? 32'hB4B4B4B5 : c0_t; // single line: 48/17 = 2.82353
    wire [15:0] c1 = (PWL == 0) ? 16'h7878     : c1_t; //              32/17 = 1.88232

    wire [31:0] cd; // c1 * d, Q2.30
    vedic_16x16 u_mul (.a(c1), .b(d[30:15]), .p(cd));
    sub32 u_sub (.a(c0), .b(cd), .diff(x0), .borrow());
endmodule
