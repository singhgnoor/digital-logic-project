`timescale 1ns/1ps

/*
sub32 -> 32 bit subtractor, hybrid adder: four 8 bit CLA blocks (cla8),
carry ripples from one block to the next (look ahead inside a block, ripple between blocks)
a - b = a + (~b) + 1   (two's complement, the +1 goes in as carry-in of the first block)
borrow = 1 when a < b  (i.e. no carry out of bit 31)
used in fp_div for the  2 - e  step of the Newton Raphson iteration
*/

module sub32 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [31:0] diff,
    output wire borrow
);
    wire [4:0] c; // carry between the blocks, c[k] is carry into block k

    assign c[0] = 1'b1; // the +1 of two's complement

    genvar k;
    generate
        for (k = 0; k < 4; k = k + 1) begin : g_cla
            cla8 u_cla (.a(a[8*k+7:8*k]), .b(~b[8*k+7:8*k]), .cin(c[k]), .sum(diff[8*k+7:8*k]), .cout(c[k+1]));
        end
    endgenerate

    assign borrow = ~c[4];
endmodule
