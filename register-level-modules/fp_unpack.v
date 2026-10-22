`timescale 1ns/1ps

/*
fp_unpack -> split IEEE754 single precision into {sign, exponent, 24bit mantissa}
             and classify the operand
STAGE 3: special case detection added (zero, subnormal, infinity, NaN)

   exp      frac    class
   0        0       zero        (+0 / -0)
   0        !=0     subnormal   (treated as zero by fp_div -> DAZ)
   1..254   any     normal      (hidden bit = 1)
   255      0       infinity
   255      !=0     NaN         (frac[22] = 1 -> quiet, 0 -> signalling)
*/

module fp_unpack (
    input  wire [31:0] x,
    output wire        sign,
    output wire [7:0]  exp,
    output wire [23:0] mant,     // {hidden, fraction}, value = mant / 2^23, in [1,2) for normals
    // STAGE 3 CHANGE START: operand classification
    output wire        is_zero,
    output wire        is_sub,
    output wire        is_inf,
    output wire        is_nan
    // STAGE 3 CHANGE END
);
    assign sign = x[31];
    assign exp  = x[30:23];

    // STAGE 3 CHANGE START: classification + correct hidden bit
    // STAGE 2 (old): assign mant = {1'b1, x[22:0]};
    wire exp_zero  = ~|x[30:23];   // exponent all 0s
    wire exp_ones  =  &x[30:23];   // exponent all 1s
    wire frac_zero = ~|x[22:0];

    assign is_zero = exp_zero &  frac_zero;
    assign is_sub  = exp_zero & ~frac_zero;
    assign is_inf  = exp_ones &  frac_zero;
    assign is_nan  = exp_ones & ~frac_zero;

    assign mant = {~exp_zero, x[22:0]};   // hidden bit is 0 for zero / subnormal
    // STAGE 3 CHANGE END
endmodule
