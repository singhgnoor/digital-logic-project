`timescale 1ns/1ps
/*
fp_round_pack -> normalize, round and pack the quotient

prod = mant_a * x  where x = 1/d  and d = mant_b/2^24
mant_a/2^23 * x/2^30 = 2*(mant_a/mant_b)  with 53 fraction bits
so prod = 2*ratio * 2^53, ratio = mant_a/mant_b in (0.5, 2)
ratio >= 1  -> leading 1 at prod[54]
ratio <  1  -> leading 1 at prod[53]
exp_base = expA - expB + 127 (int base 10, for understanding)
Overflow -> Inf, underflow -> flushed to zero (FTZ, subnormal results are not produced).
STAGE 3: only normal operands reach this block, fp_div handles zero / Inf / NaN / subnormal inputs.
*/

module fp_round_pack (
    input  wire sign,
    input  wire signed [9:0] exp_base,
    input  wire [55:0] prod,
    output wire [31:0] result,
    output wire overflow,
    output wire underflow
);
    wire ge1   = prod[54]; // checking ratio >= 1 or not
    wire [23:0] mant  = ge1 ? prod[54:31] : prod[53:30];
    wire guard = ge1 ? prod[30]    : prod[29];
    wire stky  = ge1 ? (|prod[29:0]) : (|prod[28:0]);

    wire round_up = guard & (stky | mant[0]); // rounding to nearest-even
    wire [24:0] mant_r   = {1'b0, mant} + round_up;
    wire carry    = mant_r[24]; // 1.111..1 rounded up -> 10.000..0

    wire signed [9:0] exp_f = exp_base - (ge1 ? 10'sd0 : 10'sd1) + (carry ? 10'sd1 : 10'sd0);
    wire [22:0] frac = carry ? 23'd0 : mant_r[22:0];

    assign overflow  = (exp_f >= 10'sd255);
    assign underflow = (exp_f <= 10'sd0);

    assign result = overflow  ? {sign, 8'hFF, 23'd0} :
                    underflow ? {sign, 8'h00, 23'd0} :
                                {sign, exp_f[7:0], frac};
endmodule
