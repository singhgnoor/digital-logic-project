`timescale 1ns/1ps

/*
rsqrt_lut -> look-up-table for the square root seed y0 ~ 1/sqrt(x)
x is the range reduced radicand (Q1.31), it depends on the parity of the exponent:
    idx[3] = ea[0] = 0 -> E = ea-127 odd  -> x = m/2 in [0.5, 1)    8 segments of width 1/16
    idx[3] = ea[0] = 1 -> E = ea-127 even -> x = m/4 in [0.25, 0.5) 8 segments of width 1/32
idx[2:0] = top 3 fraction bits of A (frac[22:20])
each entry = 2/(sqrt(lo) + sqrt(hi)) -> same error at both ends of the segment (minimax)
gives ~5.09 correct bits (worst segment [0.25, 0.28125))
*/

module rsqrt_lut (
    input  wire [3:0]  idx,
    output reg  [31:0] y0  // Q2.30
);
    always @(*) begin
        case (idx)
            4'd0:  y0 = 32'h57D86660;  // [0.5000, 0.5625)   1.37258
            4'd1:  y0 = 32'h53160EB7;  // [0.5625, 0.6250)   1.29822
            4'd2:  y0 = 32'h4F069649;  // [0.6250, 0.6875)   1.23478
            4'd3:  y0 = 32'h4B821517;  // [0.6875, 0.7500)   1.17982
            4'd4:  y0 = 32'h486C1702;  // [0.7500, 0.8125)   1.13160
            4'd5:  y0 = 32'h45AFB342;  // [0.8125, 0.8750)   1.08885
            4'd6:  y0 = 32'h433D27FA;  // [0.8750, 0.9375)   1.05061
            4'd7:  y0 = 32'h410853AC;  // [0.9375, 1.0000)   1.01613
            4'd8:  y0 = 32'h7C3B6670;  // [0.25000, 0.28125) 1.94113
            4'd9:  y0 = 32'h7580675C;  // [0.28125, 0.31250) 1.83596
            4'd10: y0 = 32'h6FC25EDE;  // [0.31250, 0.34375) 1.74624
            4'd11: y0 = 32'h6AC8DD5A;  // [0.34375, 0.37500) 1.66851
            4'd12: y0 = 32'h666BA585;  // [0.37500, 0.40625) 1.60032
            4'd13: y0 = 32'h628D2560;  // [0.40625, 0.43750) 1.53987
            4'd14: y0 = 32'h5F171432;  // [0.43750, 0.46875) 1.48578
            4'd15: y0 = 32'h5BF84A24;  // [0.46875, 0.50000) 1.43703
        endcase
    end
endmodule
