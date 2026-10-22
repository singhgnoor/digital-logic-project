// Combined design.sv, having all register level modules
// STAGE 3: fp_unpack / fp_div / fp_round_pack updated for special cases (NaN, Inf, zero, subnormal) + exception flags

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
    output wire        is_zero,
    output wire        is_sub,
    output wire        is_inf,
    output wire        is_nan
);
    assign sign = x[31];
    assign exp  = x[30:23];

    wire exp_zero  = ~|x[30:23];   // exponent all 0s
    wire exp_ones  =  &x[30:23];   // exponent all 1s
    wire frac_zero = ~|x[22:0];

    assign is_zero = exp_zero &  frac_zero;
    assign is_sub  = exp_zero & ~frac_zero;
    assign is_inf  = exp_ones &  frac_zero;
    assign is_nan  = exp_ones & ~frac_zero;

    assign mant = {~exp_zero, x[22:0]};   // hidden bit is 0 for zero / subnormal
endmodule


`timescale 1ns/1ps
/*
fp_round_pack -> normalize, round and pack the quotient

prod = mant_a * x  where x = 1/d  and d = mant_b/2^24
mant_a/2^23 * x/2^30 = 2*(mant_a/mant_b)  with 53 fraction bits
so prod = 2*ratio * 2^53, ratio = mant_a/mant_b in (0.5, 2)
ratio >= 1  -> leading 1 at prod[54]
ratio <  1  -> leading 1 at prod[53]
exp_base = expA - expB + 127 (int base 10, for understanding)
Overflow -> Inf, underflow -> flushed to zero (FTZ, subnormal results are not produced).
STAGE 3: only normal operands reach this block, fp_div handles zero / Inf / NaN / subnormal inputs.
*/

module fp_round_pack (
    input  wire sign,
    input  wire signed [9:0] exp_base,
    input  wire [55:0] prod,
    output wire [31:0] result,
    output wire overflow,
    output wire underflow
);
    wire ge1   = prod[54]; // checking ratio >= 1 or not
    wire [23:0] mant  = ge1 ? prod[54:31] : prod[53:30];
    wire guard = ge1 ? prod[30]    : prod[29];
    wire stky  = ge1 ? (|prod[29:0]) : (|prod[28:0]);

    wire round_up = guard & (stky | mant[0]); // rounding to nearest-even
    wire [24:0] mant_r   = {1'b0, mant} + round_up;
    wire carry    = mant_r[24]; // 1.111..1 rounded up -> 10.000..0

    wire signed [9:0] exp_f = exp_base - (ge1 ? 10'sd0 : 10'sd1) + (carry ? 10'sd1 : 10'sd0);
    wire [22:0] frac = carry ? 23'd0 : mant_r[22:0];

    assign overflow  = (exp_f >= 10'sd255);
    assign underflow = (exp_f <= 10'sd0);

    assign result = overflow  ? {sign, 8'hFF, 23'd0} :
                    underflow ? {sign, 8'h00, 23'd0} :
                                {sign, exp_f[7:0], frac};
endmodule


`timescale 1ns/1ps

// Datapath (all unsigned fixed point, ONE shared 32x32 multiplier, FSM-sequenced)
//   d  : Q1.31   (= {mant_B, 7'b0})
//   x  : Q2.30   (1.0 = 32'h4000_0000, 2.0 = 32'h8000_0000)
//   e  : Q2.30   e = d*x ~ 1.0

// States : IDLE -> (S_E -> S_X) x num_iter -> S_Q -> S_OUT -> IDLE
//   S_E : e  = d * x
//   S_X : x  = x * (2 - e)
//   S_Q : P  = mant_A * x
//   S_OUT: normalise / round / pack, raise 'done' for one cycle
// STAGE 3: special operands (NaN / Inf / zero / subnormal) bypass the NR loop:
//   IDLE --start & special--> IDLE   (result + done on the next clock edge, 1 cycle latency)

/*
fp_div -> IEEE754 single-precision divider using Newton Raphson reciprocal.
Required ans = A * (1/B)
Newton-Raphson iteration : x(n+1) = x(n) * (2-d*x(n))       
                            : d -> normalised mantissa (mantissa of B/2) in [0.5, 1)

STAGE 3: special cases handled (priority order, top wins)
   1. A or B is NaN           -> NaN (input NaN propagated and made quiet, A has priority)
   2. 0/0  or  Inf/Inf        -> default quiet NaN 0x7FC00000          (invalid)
   3. Inf/finite              -> +-Inf
   4. nonzero finite / 0      -> +-Inf                                 (div_by_zero)
   5. 0/nonzero  or  finite/Inf -> +-0
   sign of Inf / zero results = signA ^ signB
Subnormal inputs are treated as zero (DAZ) and subnormal results are flushed to zero (FTZ).

*/

module fp_div (
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [31:0] a, // dividend
    input  wire [31:0] b, // divisor
    input  wire [2:0]  num_iter, // number of newton-raphson iterations (0..7)
    output reg  [31:0] result,
    output reg done, // 1-cycle pulse, result valid
    output wire busy,
    output reg  [3:0]  flags, // {invalid, div_by_zero, overflow, underflow}
    // debug or analysis
    output wire [31:0] x_dbg, // current reciprocal estimate, for convergence monitoring
    output wire [2:0]  iter_dbg
);
    localparam IDLE = 3'd0, S_E = 3'd1, S_X = 3'd2, S_Q = 3'd3, S_OUT = 3'd4;

    reg [2:0]  state;
    reg [2:0]  iter;
    reg sign_r;
    reg [7:0]  ea_r, eb_r;
    reg [23:0] ma_r;
    reg [31:0] d_r, x_r, e_r;
    reg [55:0] prod_r;

    assign busy = (state != IDLE);
    assign x_dbg = x_r;
    assign iter_dbg = iter;

    // unpack the combination on input porta
    wire sa, sb;
    wire [7:0]  ea, eb;
    wire [23:0] ma, mb;
    wire za, suba, infa, nana;
    wire zb, subb, infb, nanb;
    fp_unpack u_a (.x(a), .sign(sa), .exp(ea), .mant(ma),
                   .is_zero(za), .is_sub(suba), .is_inf(infa), .is_nan(nana));
    fp_unpack u_b (.x(b), .sign(sb), .exp(eb), .mant(mb),
                   .is_zero(zb), .is_sub(subb), .is_inf(infb), .is_nan(nanb));

    wire a_zero = za | suba;   // DAZ: subnormal input counts as zero
    wire b_zero = zb | subb;
    wire s_q    = sa ^ sb;

    wire r_nan  = nana | nanb | (a_zero & b_zero) | (infa & infb);
    wire r_inf  = ~r_nan & (infa | b_zero);
    wire r_zero = ~r_nan & ~r_inf & (a_zero | infb);
    wire special = r_nan | r_inf | r_zero;   // any of these -> skip the NR datapath

    wire invalid_w = (nana & ~a[22]) | (nanb & ~b[22])     // signalling NaN input
                   | (a_zero & b_zero) | (infa & infb);    // 0/0, Inf/Inf
    wire dbz_w     = b_zero & ~a_zero & ~infa & ~nana & ~nanb; // finite nonzero / 0

    wire [31:0] nan_w = nana ? {a[31:23], 1'b1, a[21:0]} :   // propagate A (quietened)
                        nanb ? {b[31:23], 1'b1, b[21:0]} :   // else propagate B
                               32'h7FC0_0000;                // else default quiet NaN
    wire [31:0] special_val = r_nan ? nan_w :
                              r_inf ? {s_q, 8'hFF, 23'd0} :
                                      {s_q, 31'd0};

    // get the x0 from look up table
    wire [31:0] x0;
    seed_lut u_lut (.idx(mb[22:20]), .x0(x0));

    // using the standard multiplyier that is the there for the project
    wire [31:0] t_w; // 2 - e
    sub32 u_sub (.a(32'h8000_0000), .b(e_r), .diff(t_w), .borrow());
    reg  [31:0] mul_a, mul_b;
    wire [63:0] mul_p;

    always @(*) begin
        case (state)
            S_E:     begin mul_a = d_r;            mul_b = x_r; end
            S_X:     begin mul_a = x_r;            mul_b = t_w; end
            S_Q:     begin mul_a = {8'b0, ma_r};   mul_b = x_r; end
            default: begin mul_a = 32'd0;          mul_b = 32'd0; end
        endcase
    end
    mant_mul u_mul (.a(mul_a), .b(mul_b), .p(mul_p));

    // round to required bits and pack
    wire signed [9:0] exp_base = $signed({2'b00, ea_r}) - $signed({2'b00, eb_r}) + 10'sd127;
    wire [31:0] res_w;
    wire        ovf_w, unf_w;
    fp_round_pack u_rp (.sign(sign_r), .exp_base(exp_base), .prod(prod_r),
                        .result(res_w), .overflow(ovf_w), .underflow(unf_w));

    // controlling the cycles [FSM (Finite state machine)]
    always @(posedge clk) begin
        if (rst) begin
            state  <= IDLE;
            done   <= 1'b0;
            result <= 32'd0;
            iter   <= 3'd0;
            flags  <= 4'd0;   // STAGE 3
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    if (special) begin
                        result <= special_val;
                        flags  <= {invalid_w, dbz_w, 2'b00};
                        done   <= 1'b1;
                        state  <= IDLE;
                    end else begin
                        sign_r <= s_q;
                        ea_r   <= ea;
                        eb_r   <= eb;
                        ma_r   <= ma;
                        d_r    <= {mb, 7'b0};
                        x_r    <= x0;
                        iter   <= 3'd0;
                        state  <= (num_iter == 3'd0) ? S_Q : S_E;
                    end
                end
                S_E: begin
                    e_r   <= mul_p[62:31];
                    state <= S_X;
                end
                S_X: begin
                    x_r   <= mul_p[61:30];
                    iter  <= iter + 3'd1;
                    state <= ((iter + 3'd1) == num_iter) ? S_Q : S_E;
                end
                S_Q: begin
                    prod_r <= mul_p[55:0];
                    state  <= S_OUT;
                end
                S_OUT: begin
                    result <= res_w;
                    flags  <= {2'b00, ovf_w, unf_w};   // STAGE 3
                    done   <= 1'b1;
                    state  <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule


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


`timescale 1ns/1ps

/*
cla8 -> 8 bit carry look ahead adder, gate level
g = a & b (bit generates a carry),  p = a ^ b (bit passes the carry in to the next bit)
every carry is written directly from g, p and cin, so no carry waits for the previous one:
    c[i+1] = g[i] | p[i]&g[i-1] | p[i]&p[i-1]&g[i-2] | ... | p[i]&...&p[0]&cin
sum = p ^ c
building block of sub32 (4 of these, carry rippling between the blocks)
*/

module cla8 (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire cin,
    output wire [7:0] sum,
    output wire cout
);
    wire [7:0] g = a & b; // generate
    wire [7:0] p = a ^ b; // propagate
    wire [8:0] c;

    assign c[0] = cin;
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & g[0]) | (&p[1:0] & cin);
    assign c[3] = g[2] | (p[2] & g[1]) | (&p[2:1] & g[0]) | (&p[2:0] & cin);
    assign c[4] = g[3] | (p[3] & g[2]) | (&p[3:2] & g[1]) | (&p[3:1] & g[0]) | (&p[3:0] & cin);
    assign c[5] = g[4] | (p[4] & g[3]) | (&p[4:3] & g[2]) | (&p[4:2] & g[1]) | (&p[4:1] & g[0])
                | (&p[4:0] & cin);
    assign c[6] = g[5] | (p[5] & g[4]) | (&p[5:4] & g[3]) | (&p[5:3] & g[2]) | (&p[5:2] & g[1])
                | (&p[5:1] & g[0]) | (&p[5:0] & cin);
    assign c[7] = g[6] | (p[6] & g[5]) | (&p[6:5] & g[4]) | (&p[6:4] & g[3]) | (&p[6:3] & g[2])
                | (&p[6:2] & g[1]) | (&p[6:1] & g[0]) | (&p[6:0] & cin);
    assign c[8] = g[7] | (p[7] & g[6]) | (&p[7:6] & g[5]) | (&p[7:5] & g[4]) | (&p[7:4] & g[3])
                | (&p[7:3] & g[2]) | (&p[7:2] & g[1]) | (&p[7:1] & g[0]) | (&p[7:0] & cin);

    assign sum  = p ^ c[7:0];
    assign cout = c[8];
endmodule


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
