`timescale 1ns/1ps

/*
fp_unpack -> split IEEE754 single precision into {sing, exponent, 24bit mantissa}
Stage2 unpacks normal numbers only!!!
Stage3 plan will implement special case detection i.e. zero, infinity, other cases
*/

module fp_unpack (
    input  wire [31:0] x,
    output wire        sign,
    output wire [7:0]  exp,
    output wire [23:0] mant      // {1, fraction}, value = mant / 2^23, in [1,2)
);
    assign sign = x[31];
    assign exp  = x[30:23];
    assign mant = {1'b1, x[22:0]};
endmodule
