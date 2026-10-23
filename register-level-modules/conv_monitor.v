// STAGE 4 - NEW FILE
`timescale 1ns/1ps

/*
conv_monitor -> residual based convergence monitor
e = d*x (divide) or x*y*y (sqrt) is already made by the shared multiplier, ~1.0 in Q2.30
r = e - 1.0 is the current relative error (two's complement, through sub32)
|r| < 2^-k  <=>  bits r[31:30-k] are all 0s (small positive) or all 1s (small negative)
conv   : |r| < 2^-15 -> the error squares every iteration, so after THIS update x is ~2^-30 good,
         this iteration can be the last one (no extra cycle needed)
res_ok : |r| < 2^-26 -> used on the final x as a fault check (fault free runs stay below ~2^-30)
*/

module conv_monitor (
    input  wire [31:0] e,      // Q2.30, ~1.0
    output wire conv,          // |e - 1| < 2^-15
    output wire res_ok         // |e - 1| < 2^-26
);
    wire [31:0] r; // e - 1.0
    sub32 u_sub (.a(e), .b(32'h4000_0000), .diff(r), .borrow());

    assign conv   = (~|r[31:15]) | (&r[31:15]);
    assign res_ok = (~|r[31:4])  | (&r[31:4]);
endmodule
