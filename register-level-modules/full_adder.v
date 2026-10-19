// STAGE 3 - NEW FILE
`timescale 1ns/1ps

/*
full_adder -> 1 bit full adder, gate level
basic cell of sub32 (and of the CLA later)
*/

module full_adder (
    input  wire a,
    input  wire b,
    input  wire cin,
    output wire sum,
    output wire cout
);
    assign sum  = a ^ b ^ cin;
    assign cout = (a & b) | (cin & (a ^ b));
endmodule
