`timescale 1ns/1ps

/*
mant-mul -> Mantissa multiplication 
shared multiplication module (everything in theh project multiplies through
 this), in stage2 uses provided multiplication module  have to design the 
compete module (gate-level architecture) in stage3 keeping the ports same. 
*/

module mant_mul (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [63:0] p
);
    assign p = a * b;
endmodule
