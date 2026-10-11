`timescale 1ns/1ps

/*
seed_lut -> look-up-table (initial approximation for x0)
using the top 3 bits of the normalised mantissa (from 0.5 closed to 1 open)
We have 8 intervals with width 1/16, so each entry in that for minimum error
comes out to be -> 2/(low + high) [NOT THE MEAN][For minimum error]
gives ~~4 corrected bits of intial approximation
*/

module seed_lut (
    input  wire [2:0]  idx,
    output reg  [31:0] x0
);
    always @(*) begin
        case (idx)
            3'd0: x0 = 32'h78787878;  // 32/17 = 1.88235
            3'd1: x0 = 32'h6BCA1AF3;  // 32/19 = 1.68421
            3'd2: x0 = 32'h61861862;  // 32/21 = 1.52381
            3'd3: x0 = 32'h590B2164;  // 32/23 = 1.39130
            3'd4: x0 = 32'h51EB851F;  // 32/25 = 1.28000
            3'd5: x0 = 32'h4BDA12F7;  // 32/27 = 1.18519
            3'd6: x0 = 32'h469EE584;  // 32/29 = 1.10345
            3'd7: x0 = 32'h42108421;  // 32/31 = 1.03226
        endcase
    end
endmodule
