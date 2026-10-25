// Combined design.sv, having all register level modules
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
    parameter USE_VEDIC = 1; // 1 -> vedic_32x32 (default), 0 -> behavioural '*'

    generate
        if (USE_VEDIC) begin : g_vedic
            vedic_32x32 u_vedic (.a(a), .b(b), .p(p));
        end else begin : g_star
            assign p = a * b;
        end
    endgenerate
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
seed_linear -> linear / piecewise linear seed for x0 = 1/d   (replaces the LUT when SEED_MODE = 1 or 2)
x0 = c0 - c1*d,  best (minimax) line on [a, b] for the relative error |d*x0 - 1|:
    c1 = 2 / (ab + (a+b)^2/4)      c0 = c1*(a+b)      worst error = (b-a)^2 / ((a+b)^2 + 4ab)
PWL = 0 : one line on [0.5, 1)  -> x0 = 48/17 - 32/17*d, error 1/17   (4.09 bits, same as the 8 entry LUT)
PWL = 1 : 8 lines, segment = d[29:27] = mb[22:20] (same index as the LUT), error ~2^-9.17
          -> 9.15 bits after the 16 bit truncation (checked on all 2^23 mantissas in the model), so 2 NR iterations are enough

Q formats
    d   : Q1.31, only d[30:15] is used (Q0.16, d[31] is always 0, d[30] is the hidden 1)
    c1  : Q2.14 (16 bit, max 3.55)
    c1*d: Q2.14 * Q0.16 = Q2.30, straight from vedic_16x16, no shift needed (value < 2)
    c0  : Q2.30 (max 3.77, fits below 4)
    x0  : c0 - c1*d through sub32, Q2.30
combinational (gate level multiplier + CLA subtractor), so it needs no extra FSM cycle
*/

module seed_linear (
    input  wire [31:0] d,  // Q1.31 divisor, in [0.5, 1)
    output wire [31:0] x0  // Q2.30 seed
);
    parameter PWL = 1; // 0 -> single line, 1 -> 8 segment piecewise linear

    reg [31:0] c0_t; // Q2.30, 8 segment table
    reg [15:0] c1_t; // Q2.14

    always @(*) begin
        case (d[29:27])
            3'd0: begin c0_t = 32'hF15BD72F; c1_t = 16'hE329; end  // [0.5000, 0.5625)  3.77123 - 3.54938 d
            3'd1: begin c0_t = 32'hD7E0C127; c1_t = 16'hB5CB; end  // [0.5625, 0.6250)  3.37309 - 2.84052 d
            3'd2: begin c0_t = 32'hC344DE01; c1_t = 16'h94C7; end  // [0.6250, 0.6875)  3.05108 - 2.32465 d
            3'd3: begin c0_t = 32'hB2416483; c1_t = 16'h7C01; end  // [0.6875, 0.7500)  2.78524 - 1.93756 d
            3'd4: begin c0_t = 32'hA3F89F0D; c1_t = 16'h68F1; end  // [0.7500, 0.8125)  2.56205 - 1.63971 d
            3'd5: begin c0_t = 32'h97CECD93; c1_t = 16'h59F6; end  // [0.8125, 0.8750)  2.37200 - 1.40564 d
            3'd6: begin c0_t = 32'h8D534D84; c1_t = 16'h4DF9; end  // [0.8750, 0.9375)  2.20821 - 1.21832 d
            3'd7: begin c0_t = 32'h8432A3EA; c1_t = 16'h443B; end  // [0.9375, 1.0000)  2.06559 - 1.06610 d
        endcase
    end

    wire [31:0] c0 = (PWL == 0) ? 32'hB4B4B4B5 : c0_t; // single line: 48/17 = 2.82353
    wire [15:0] c1 = (PWL == 0) ? 16'h7878     : c1_t; //              32/17 = 1.88232

    wire [31:0] cd; // c1 * d, Q2.30
    vedic_16x16 u_mul (.a(c1), .b(d[30:15]), .p(cd));
    sub32 u_sub (.a(c0), .b(cd), .diff(x0), .borrow());
endmodule


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


`timescale 1ns/1ps

/*
fp_unpack -> split IEEE754 single precision into {sign, exponent, 24bit mantissa}
             and classify the operand

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

/*
sqrt_pack -> exact rounding + pack for the square root
root = x*y in [0.5, 1), q = its top 24 bits (truncated), that is the mantissa 1.xxx before rounding
true root scaled like q: S = sqrt(x) * 2^24, so S^2 = x * 2^48, compare it with the midpoint q + 1/2:
    S > q + 1/2   <=>   4*x*2^48 > (2q+1)^2       (both are integers, never equal -> no ties in sqrt)
the shared multiplier makes (2q+1)^2 in S_OUT, chk = {q, 1} = 2q+1 goes out to it
4*x*2^48 = x_q131 * 2^19 has its low 25 bits = 0 and (2q+1)^2 is odd, so comparing bits [49:25] is enough:
    round_up = (sq[49:25] < x[30:6])  -> borrow of sub32
this needs |hw root - true root| < 1/2 ULP, true with >= 3 NR iterations (all 2^24 cases checked in the model)
exponent : biased result = (ea + 127) >> 1   (ea odd -> E even, ea even -> E odd, the >>1 does the floor)
no overflow / underflow possible for sqrt
*/

module sqrt_pack (
    input  wire [7:0]  exp_a,   // biased exponent of A
    input  wire [31:0] x,       // Q1.31 radicand (m/4 or m/2)
    input  wire [23:0] q,       // truncated root mantissa
    input  wire [24:0] sq_hi,   // (2q+1)^2 bits [49:25], from the shared multiplier
    output wire [24:0] chk,     // 2q+1, to the shared multiplier
    output wire [31:0] result
);
    assign chk = {q, 1'b1}; // 2q + 1

    wire round_up; // borrow = 1 when sq_hi < x[30:6]
    sub32 u_sub (.a({7'd0, sq_hi}), .b({7'd0, x[30:6]}), .diff(), .borrow(round_up));

    wire [24:0] mant  = {1'b0, q} + round_up;
    wire carry = mant[24];  // 1.111..1 rounded up -> 10.000..0
    wire [8:0]  esum  = {1'b0, exp_a} + 9'd127;
    wire [7:0]  exp_f = esum[8:1] + carry;
    wire [22:0] frac  = carry ? 23'd0 : mant[22:0];

    assign result = {1'b0, exp_f, frac};
endmodule


`timescale 1ns/1ps

/*
conv_monitor -> residual based convergence monitor
e = d*x (divide) or x*y*y (sqrt) is already made by the shared multiplier, ~1.0 in Q2.30
r = e - 1.0 is the current relative error (two's complement, through sub32)
|r| < 2^-k  <=>  bits r[31:30-k] are all 0s (small positive) or all 1s (small negative)
conv   : |r| < 2^-15 -> the error squares every iteration, so after THIS update x is ~2^-30 good,
         this iteration can be the last one (no extra cycle needed)
res_ok : |r| < 2^-26 -> used on the final x as a fault check (fault free runs stay below ~2^-30)
*/

module conv_monitor (
    input  wire [31:0] e,      // Q2.30, ~1.0
    output wire conv,          // |e - 1| < 2^-15
    output wire res_ok         // |e - 1| < 2^-26
);
    wire [31:0] r; // e - 1.0
    sub32 u_sub (.a(e), .b(32'h4000_0000), .diff(r), .borrow());

    assign conv   = (~|r[31:15]) | (&r[31:15]);
    assign res_ok = (~|r[31:4])  | (&r[31:4]);
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
//   IDLE --start & special--> IDLE   (result + done on the next clock edge, 1 cycle latency)
//   IDLE -> (S_Y2 -> S_XY2 -> S_YT) x num_iter -> S_SQ -> S_OUT -> IDLE
//   S_Y2 : e = y * y
//   S_XY2: e = x * (y*y)            (residual for the monitor)
//   S_YT : y = y * (3 - x*y*y) / 2  (the /2 is a one bit bigger shift)
//   S_SQ : root = x * y
//   S_OUT: exact rounding check (2q+1)^2 vs 4x on the shared multiplier, pack

/*
fp_div -> IEEE754 single-precision divider using Newton Raphson reciprocal.
Required ans = A * (1/B)
Newton-Raphson iteration : x(n+1) = x(n) * (2-d*x(n))
                            : d -> normalised mantissa (mantissa of B/2) in [0.5, 1)

Special cases handled (priority order, top wins)
   1. A or B is NaN           -> NaN (input NaN propagated and made quiet, A has priority)
   2. 0/0  or  Inf/Inf        -> default quiet NaN 0x7FC00000          (invalid)
   3. Inf/finite              -> +-Inf
   4. nonzero finite / 0      -> +-Inf                                 (div_by_zero)
   5. 0/nonzero  or  finite/Inf -> +-0
   sign of Inf / zero results = signA ^ signB
Subnormal inputs are treated as zero (DAZ) and subnormal results are flushed to zero (FTZ).

The module now ALSO does square root (op = 1), name kept as fp_div so nothing else breaks
Newton-Raphson for 1/sqrt : y(n+1) = y(n) * (3 - x*y(n)^2) / 2,   sqrt(x) = x * y   (no division needed)
   A = m * 2^E,  E even -> x = m/4 in [0.25, 0.5),  E odd -> x = m/2 in [0.5, 1)
   sqrt special cases: NaN -> NaN (quietened), -x -> 0x7FC00000 (invalid), +-0 -> +-0,
                       +Inf -> +Inf, subnormal -> +-0 (DAZ), no overflow / underflow possible
*/

module fp_div (
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [31:0] a, // dividend
    input  wire [31:0] b, // divisor
    input  wire [2:0]  num_iter, // number of newton-raphson iterations (0..7)
    input  wire op,        // 0 -> a / b,  1 -> sqrt(a)  (b is ignored)
    input  wire auto_stop, // 1 -> stop when the residual is small, num_iter is then the max
    output reg  [31:0] result,
    output reg done, // 1-cycle pulse, result valid
    output wire busy,
    output reg  [3:0]  flags, // {invalid, div_by_zero, overflow, underflow}
    // debug or analysis
    output wire [31:0] x_dbg, // current reciprocal estimate, for convergence monitoring
    output wire [2:0]  iter_dbg,
    output wire [2:0]  iters_used, // iterations actually done, valid with done
    output reg  converged,         // 1 -> the monitor stopped the loop before num_iter
    output reg  fault_det          // 1 -> divide result failed the final |1 - d*x| < 2^-26 check
);
    parameter SEED_MODE = 0; // 0 -> seed_lut (stage 3), 1 -> single line, 2 -> piecewise linear (8 segments)

    localparam IDLE = 4'd0, S_E = 4'd1, S_X = 4'd2, S_Q = 4'd3, S_OUT = 4'd4;
    localparam S_Y2 = 4'd5, S_XY2 = 4'd6, S_YT = 4'd7, S_SQ = 4'd8; // sqrt

    reg [3:0]  state;
    reg op_r;          // 1 -> current job is a sqrt
    reg conv_r;        // monitor result of this iteration
    reg [23:0] root_r; // truncated sqrt mantissa
    reg [2:0]  iter;
    reg sign_r;
    reg [7:0]  ea_r, eb_r;
    reg [23:0] ma_r;
    reg [31:0] d_r, x_r, e_r;
    reg [55:0] prod_r;

    assign busy = (state != IDLE);
    assign x_dbg = x_r;
    assign iter_dbg = iter;
    assign iters_used = iter;

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

    wire sq_special = nana | a_zero | infa | sa;               // anything but a positive normal
    wire sq_invalid = (nana & ~a[22]) | (sa & ~a_zero & ~nana); // sNaN, sqrt(negative)
    wire [31:0] sq_special_val = nana   ? {a[31:23], 1'b1, a[21:0]} : // NaN propagated (quietened)
                                 a_zero ? {sa, 31'd0} :               // +-0 (subnormal -> DAZ)
                                 sa     ? 32'h7FC0_0000 :             // sqrt(negative) -> default NaN
                                          32'h7F80_0000;              // sqrt(+Inf) = +Inf

    // get the x0 from look up table
    wire [31:0] x0;
    generate
        if (SEED_MODE == 0) begin : g_lut
            seed_lut u_lut (.idx(mb[22:20]), .x0(x0));
        end else begin : g_lin
            seed_linear #(.PWL(SEED_MODE == 2)) u_lin (.d({1'b0, mb, 7'b0}), .x0(x0));
        end
    endgenerate

    // sqrt: range reduced radicand (by exponent parity) and its 1/sqrt seed
    wire [31:0] xs = ea[0] ? {2'b00, ma, 6'b0}   // ea odd  -> E even -> x = m/4, Q1.31
                           : {1'b0,  ma, 7'b0};  // ea even -> E odd  -> x = m/2
    wire [31:0] y0;
    rsqrt_lut u_rlut (.idx({ea[0], ma[22:20]}), .y0(y0));

    // using the standard multiplyier that is the there for the project
    wire [31:0] t_w; // 2 - e
    sub32 u_sub (.a(op_r ? 32'hC000_0000 : 32'h8000_0000), .b(e_r), .diff(t_w), .borrow()); // 2 - e  or  3 - x*y*y
    reg  [31:0] mul_a, mul_b;
    wire [63:0] mul_p;
    wire [24:0] chk_w; // STAGE 4: 2q+1 for the sqrt rounding check

    always @(*) begin
        case (state)
            S_E, S_SQ: begin mul_a = d_r;            mul_b = x_r; end  // d*x  or  x*y
            S_X, S_YT: begin mul_a = x_r;            mul_b = t_w; end  // x*(2-e)  or  y*(3-x*y*y)
            S_Q:       begin mul_a = {8'b0, ma_r};   mul_b = x_r; end
            S_Y2:      begin mul_a = x_r;            mul_b = x_r; end  // y*y
            S_XY2:     begin mul_a = d_r;            mul_b = e_r; end  // x*(y*y)
            S_OUT:     begin mul_a = op_r ? {7'b0, chk_w} : d_r;       // (2q+1)^2  or  d*x (residual)
                             mul_b = op_r ? {7'b0, chk_w} : x_r; end
            default:   begin mul_a = 32'd0;          mul_b = 32'd0; end
        endcase
    end
    mant_mul u_mul (.a(mul_a), .b(mul_b), .p(mul_p));

    wire conv_w, res_ok_w;
    conv_monitor u_mon (.e(mul_p[62:31]), .conv(conv_w), .res_ok(res_ok_w)); // e = d*x or x*y*y

    wire last_w  = ((iter + 3'd1) == num_iter);   // cap reached
    wire early_w = auto_stop & conv_r & ~last_w;  // monitor says this update is good enough

    // round to required bits and pack
    wire signed [9:0] exp_base = $signed({2'b00, ea_r}) - $signed({2'b00, eb_r}) + 10'sd127;
    wire [31:0] res_w;
    wire        ovf_w, unf_w;
    fp_round_pack u_rp (.sign(sign_r), .exp_base(exp_base), .prod(prod_r),
                        .result(res_w), .overflow(ovf_w), .underflow(unf_w));

    wire [31:0] sq_res_w;
    sqrt_pack u_sp (.exp_a(ea_r), .x(d_r), .q(root_r), .sq_hi(mul_p[49:25]),
                    .chk(chk_w), .result(sq_res_w));

    // the testbench sets dut.fi_en / fi_sel / fi_iter / fi_bit, one bit is flipped while the register is written
    // fi_sel : 0 = x, 1 = e, 2 = d, 3 = mant_A, 4 = prod      (divide only)
    // fi_iter: x -> x_k (k=0 is the seed, k = value after the k-th iteration),  e -> e of iteration k (= d*x_k)
    //          d, mant_A, prod are written once, fi_iter is not used for them
`ifdef FAULT_INJ
    reg fi_en;
    reg [2:0] fi_sel, fi_iter;
    reg [5:0] fi_bit;
    initial begin fi_en = 1'b0; fi_sel = 3'd0; fi_iter = 3'd0; fi_bit = 6'd0; end

    wire [63:0] fi_m = 64'd1 << fi_bit;
    wire [2:0]  fi_k = (state == IDLE) ? 3'd0 : iter + 3'd1; // index of the x being written
    wire [31:0] fi_x = (fi_en && fi_sel == 3'd0 && fi_iter == fi_k) ? fi_m[31:0] : 32'd0;
    wire [31:0] fi_e = (fi_en && fi_sel == 3'd1 && fi_iter == iter) ? fi_m[31:0] : 32'd0;
    wire [31:0] fi_d = (fi_en && fi_sel == 3'd2) ? fi_m[31:0] : 32'd0;
    wire [23:0] fi_a = (fi_en && fi_sel == 3'd3) ? fi_m[23:0] : 24'd0;
    wire [55:0] fi_p = (fi_en && fi_sel == 3'd4) ? fi_m[55:0] : 56'd0;
`else
    wire [31:0] fi_x = 32'd0, fi_e = 32'd0, fi_d = 32'd0; // normal build: no fault, XOR with 0
    wire [23:0] fi_a = 24'd0;
    wire [55:0] fi_p = 56'd0;
`endif

    // controlling the cycles [FSM (Finite state machine)]
    always @(posedge clk) begin
        if (rst) begin
            state  <= IDLE;
            done   <= 1'b0;
            result <= 32'd0;
            iter   <= 3'd0;
            flags  <= 4'd0;
            op_r      <= 1'b0;
            conv_r    <= 1'b0;
            converged <= 1'b0;
            fault_det <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    op_r      <= op;
                    iter      <= 3'd0;
                    converged <= 1'b0;
                    fault_det <= 1'b0;
                    if (op ? sq_special : special) begin
                        result <= op ? sq_special_val : special_val;
                        flags  <= op ? {sq_invalid, 3'b000} : {invalid_w, dbz_w, 2'b00};
                        done   <= 1'b1;
                        state  <= IDLE;
                    end else begin
                        sign_r <= s_q;
                        ea_r   <= ea;
                        eb_r   <= eb;
                        ma_r   <= ma ^ fi_a;
                        d_r    <= op ? xs : ({mb, 7'b0} ^ fi_d);
                        x_r    <= op ? y0 : (x0 ^ fi_x);
                        state  <= (num_iter == 3'd0) ? (op ? S_SQ : S_Q) : (op ? S_Y2 : S_E);
                    end
                end
                S_E: begin
                    e_r    <= mul_p[62:31] ^ fi_e;
                    conv_r <= conv_w;
                    state  <= S_X;
                end
                S_X: begin
                    x_r       <= mul_p[61:30] ^ fi_x;
                    iter      <= iter + 3'd1;
                    converged <= early_w;
                    state     <= (early_w | last_w) ? S_Q : S_E;
                end
                S_Q: begin
                    prod_r <= mul_p[55:0] ^ fi_p;
                    state  <= S_OUT;
                end
                // sqrt
                S_Y2: begin
                    e_r   <= mul_p[61:30];     // y*y, Q2.30
                    state <= S_XY2;
                end
                S_XY2: begin
                    e_r    <= mul_p[62:31];    // x*y*y, Q2.30, ~1.0
                    conv_r <= conv_w;
                    state  <= S_YT;
                end
                S_YT: begin
                    x_r       <= mul_p[62:31]; // y*t/2, Q2.30
                    iter      <= iter + 3'd1;
                    converged <= early_w;
                    state     <= (early_w | last_w) ? S_SQ : S_Y2;
                end
                S_SQ: begin
                    root_r <= mul_p[60:37];    // x*y is Q3.61 in [0.5, 1), top 24 bits
                    state  <= S_OUT;
                end
                S_OUT: begin
                    result    <= op_r ? sq_res_w : res_w;
                    flags     <= op_r ? 4'd0 : {2'b00, ovf_w, unf_w};
                    fault_det <= ~op_r & ~res_ok_w;
                    done      <= 1'b1;
                    state     <= IDLE;
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
    assign c[5] = g[4] | (p[4] & g[3]) | (&p[4:3] & g[2]) | (&p[4:2] & g[1]) | (&p[4:1] & g[0])| (&p[4:0] & cin);
    assign c[6] = g[5] | (p[5] & g[4]) | (&p[5:4] & g[3]) | (&p[5:3] & g[2]) | (&p[5:2] & g[1])| (&p[5:1] & g[0]) | (&p[5:0] & cin);
    assign c[7] = g[6] | (p[6] & g[5]) | (&p[6:5] & g[4]) | (&p[6:4] & g[3]) | (&p[6:3] & g[2])| (&p[6:2] & g[1]) | (&p[6:1] & g[0]) | (&p[6:0] & cin);
    assign c[8] = g[7] | (p[7] & g[6]) | (&p[7:6] & g[5]) | (&p[7:5] & g[4]) | (&p[7:4] & g[3])| (&p[7:3] & g[2]) | (&p[7:2] & g[1]) | (&p[7:1] & g[0]) | (&p[7:0] & cin);

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
