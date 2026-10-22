`timescale 1ns/1ps

/*
vedic_mul -> Vedic multiplier, Urdhva Tiryagbhyam ("vertically and crosswise")
Base case vedic_2x2 is gate level (4 AND + 2 half adders).
Every bigger block splits each operand into high/low halves and uses 4 smaller blocks:
    q0 = aL*bL  (vertical)    q1 = aH*bL, q2 = aL*bH  (crosswise)    q3 = aH*bH  (vertical)
    P  = q0 + ((q1 + q2) << n/2) + (q3 << n)
vedic_32x32 -> used by mant_mul (v1 divider)
vedic_24x24 -> 3 digit (8 bit) crosswise version for the FP mantissa (for v2 fp_mul, not wired yet)

The combining adders use '+' for now, can be swapped for the CLA adders later.
*/

module ha (
    input  wire a,
    input  wire b,
    output wire sum,
    output wire carry
);
    assign sum   = a ^ b;
    assign carry = a & b;
endmodule


module vedic_2x2 (
    input  wire [1:0] a,
    input  wire [1:0] b,
    output wire [3:0] p
);
    wire c1;

    assign p[0] = a[0] & b[0]; // vertical
    ha u_h0 (.a(a[1] & b[0]), .b(a[0] & b[1]), .sum(p[1]), .carry(c1));   // crosswise
    ha u_h1 (.a(a[1] & b[1]), .b(c1),          .sum(p[2]), .carry(p[3])); // vertical + carry
endmodule


module vedic_4x4 (
    input  wire [3:0] a,
    input  wire [3:0] b,
    output wire [7:0] p
);
    wire [3:0] q0, q1, q2, q3;

    vedic_2x2 u_m0 (.a(a[1:0]), .b(b[1:0]), .p(q0));
    vedic_2x2 u_m1 (.a(a[3:2]), .b(b[1:0]), .p(q1));
    vedic_2x2 u_m2 (.a(a[1:0]), .b(b[3:2]), .p(q2));
    vedic_2x2 u_m3 (.a(a[3:2]), .b(b[3:2]), .p(q3));

    wire [4:0] mid = q1 + q2; // one extra bit for the carry
    assign p = q0 + (mid << 2) + (q3 << 4);
endmodule


module vedic_8x8 (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    wire [7:0] q0, q1, q2, q3;

    vedic_4x4 u_m0 (.a(a[3:0]), .b(b[3:0]), .p(q0));
    vedic_4x4 u_m1 (.a(a[7:4]), .b(b[3:0]), .p(q1));
    vedic_4x4 u_m2 (.a(a[3:0]), .b(b[7:4]), .p(q2));
    vedic_4x4 u_m3 (.a(a[7:4]), .b(b[7:4]), .p(q3));

    wire [8:0] mid = q1 + q2;
    assign p = q0 + (mid << 4) + (q3 << 8);
endmodule


module vedic_16x16 (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [31:0] p
);
    wire [15:0] q0, q1, q2, q3;

    vedic_8x8 u_m0 (.a(a[7:0]),  .b(b[7:0]),  .p(q0));
    vedic_8x8 u_m1 (.a(a[15:8]), .b(b[7:0]),  .p(q1));
    vedic_8x8 u_m2 (.a(a[7:0]),  .b(b[15:8]), .p(q2));
    vedic_8x8 u_m3 (.a(a[15:8]), .b(b[15:8]), .p(q3));

    wire [16:0] mid = q1 + q2;
    assign p = q0 + (mid << 8) + (q3 << 16);
endmodule


module vedic_32x32 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [63:0] p
);
    wire [31:0] q0, q1, q2, q3;

    vedic_16x16 u_m0 (.a(a[15:0]),  .b(b[15:0]),  .p(q0));
    vedic_16x16 u_m1 (.a(a[31:16]), .b(b[15:0]),  .p(q1));
    vedic_16x16 u_m2 (.a(a[15:0]),  .b(b[31:16]), .p(q2));
    vedic_16x16 u_m3 (.a(a[31:16]), .b(b[31:16]), .p(q3));

    wire [32:0] mid = q1 + q2;
    assign p = q0 + (mid << 16) + (q3 << 32);
endmodule


/*
vedic_24x24 -> each operand as 3 digits of 8 bits, a = {aH, aM, aL}
nine vedic_8x8, one column at a time (same as 123 x 456 by hand):
    col0 = aL*bL
    col1 = aM*bL + aL*bM
    col2 = aH*bL + aM*bM + aL*bH
    col3 = aH*bM + aM*bH
    col4 = aH*bH
    P = col0 + (col1 << 8) + (col2 << 16) + (col3 << 24) + (col4 << 32)
*/

module vedic_24x24 (
    input  wire [23:0] a,
    input  wire [23:0] b,
    output wire [47:0] p
);
    wire [15:0] ll, ml, lm, hl, mm, lh, hm, mh, hh; // first letter -> digit of a, second -> digit of b

    vedic_8x8 u_ll (.a(a[7:0]),   .b(b[7:0]),   .p(ll));
    vedic_8x8 u_ml (.a(a[15:8]),  .b(b[7:0]),   .p(ml));
    vedic_8x8 u_lm (.a(a[7:0]),   .b(b[15:8]),  .p(lm));
    vedic_8x8 u_hl (.a(a[23:16]), .b(b[7:0]),   .p(hl));
    vedic_8x8 u_mm (.a(a[15:8]),  .b(b[15:8]),  .p(mm));
    vedic_8x8 u_lh (.a(a[7:0]),   .b(b[23:16]), .p(lh));
    vedic_8x8 u_hm (.a(a[23:16]), .b(b[15:8]),  .p(hm));
    vedic_8x8 u_mh (.a(a[15:8]),  .b(b[23:16]), .p(mh));
    vedic_8x8 u_hh (.a(a[23:16]), .b(b[23:16]), .p(hh));

    wire [16:0] col1 = ml + lm;      // 2 terms -> 1 extra bit
    wire [17:0] col2 = hl + mm + lh; // 3 terms -> 2 extra bits
    wire [16:0] col3 = hm + mh;
    assign p = ll + (col1 << 8) + (col2 << 16) + (col3 << 24) + (hh << 32);
endmodule
