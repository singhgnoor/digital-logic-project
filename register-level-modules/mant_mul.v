// STAGE 3 - MODIFIED (see markers below)
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
    // STAGE 3 CHANGE START: Vedic multiplier, behavioural '*' kept selectable for comparison
    // STAGE 2 (old): assign p = a * b;
    parameter USE_VEDIC = 1; // 1 -> vedic_32x32 (default), 0 -> behavioural '*'

    generate
        if (USE_VEDIC) begin : g_vedic
            vedic_32x32 u_vedic (.a(a), .b(b), .p(p));
        end else begin : g_star
            assign p = a * b;
        end
    endgenerate
    // STAGE 3 CHANGE END
endmodule
