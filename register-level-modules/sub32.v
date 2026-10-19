// STAGE 3 - NEW FILE
`timescale 1ns/1ps

/*
sub32 -> 32 bit subtractor made of 32 chained full adders (ripple carry)
a - b = a + (~b) + 1   (two's complement, the +1 goes in as carry-in of bit 0)
borrow = 1 when a < b  (i.e. no carry out of bit 31)
used in fp_div for the  2 - e  step of the Newton Raphson iteration
*/

module sub32 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [31:0] diff,
    output wire borrow
);
    wire [32:0] c; // carry chain, c[i] is carry into bit i

    assign c[0] = 1'b1; // the +1 of two's complement

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : g_fa
            full_adder u_fa (.a(a[i]), .b(~b[i]), .cin(c[i]), .sum(diff[i]), .cout(c[i+1]));
        end
    endgenerate

    assign borrow = ~c[32];
endmodule
