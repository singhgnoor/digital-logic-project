// STAGE 4 - NEW FILE
`timescale 1ns/1ps

/*
sqrt_pack -> exact rounding + pack for the square root
root = x*y in [0.5, 1), q = its top 24 bits (truncated), that is the mantissa 1.xxx before rounding
true root scaled like q: S = sqrt(x) * 2^24, so S^2 = x * 2^48, compare it with the midpoint q + 1/2:
    S > q + 1/2   <=>   4*x*2^48 > (2q+1)^2       (both are integers, never equal -> no ties in sqrt)
the shared multiplier makes (2q+1)^2 in S_OUT, chk = {q, 1} = 2q+1 goes out to it
4*x*2^48 = x_q131 * 2^19 has its low 25 bits = 0 and (2q+1)^2 is odd, so comparing bits [49:25] is enough:
    round_up = (sq[49:25] < x[30:6])  -> borrow of sub32
this needs |hw root - true root| < 1/2 ULP, true with >= 3 NR iterations (all 2^24 cases checked in the model)
exponent : biased result = (ea + 127) >> 1   (ea odd -> E even, ea even -> E odd, the >>1 does the floor)
no overflow / underflow possible for sqrt
*/

module sqrt_pack (
    input  wire [7:0]  exp_a,   // biased exponent of A
    input  wire [31:0] x,       // Q1.31 radicand (m/4 or m/2)
    input  wire [23:0] q,       // truncated root mantissa
    input  wire [24:0] sq_hi,   // (2q+1)^2 bits [49:25], from the shared multiplier
    output wire [24:0] chk,     // 2q+1, to the shared multiplier
    output wire [31:0] result
);
    assign chk = {q, 1'b1}; // 2q + 1

    wire round_up; // borrow = 1 when sq_hi < x[30:6]
    sub32 u_sub (.a({7'd0, sq_hi}), .b({7'd0, x[30:6]}), .diff(), .borrow(round_up));

    wire [24:0] mant  = {1'b0, q} + round_up;
    wire carry = mant[24];  // 1.111..1 rounded up -> 10.000..0
    wire [8:0]  esum  = {1'b0, exp_a} + 9'd127;
    wire [7:0]  exp_f = esum[8:1] + carry;
    wire [22:0] frac  = carry ? 23'd0 : mant[22:0];

    assign result = {1'b0, exp_f, frac};
endmodule
