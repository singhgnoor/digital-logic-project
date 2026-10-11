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
    assign p = a * b;
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
fp_unpack -> split IEEE754 single precision into {sing, exponent, 24bit mantissa}
Stage2 unpacks normal numbers only!!!
Stage3 plan will implement special case detection i.e. zero, infinity, other cases
*/

module fp_unpack (
    input  wire [31:0] x,
    output wire        sign,
    output wire [7:0]  exp,
    output wire [23:0] mant      // {1, fraction}, value = mant / 2^23, in [1,2)
);
    assign sign = x[31];
    assign exp  = x[30:23];
    assign mant = {1'b1, x[22:0]};
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
Overflow -> Inf, underflow -> flushed to zero (subnormal results not supported yet).
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

/*
fp_div -> IEEE754 single-precision divider using Newton Raphson reciprocal.
Required ans = A * (1/B)
Newton-Raphson iteration : x(n+1) = x(n) * (2-d*x(n))       
                            : d -> normalised mantissa (mantissa of B/2) in [0.5, 1)

Stage2 scope is normal inputs only (Special cases zero, inf, other not handeled yet)

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
    fp_unpack u_a (.x(a), .sign(sa), .exp(ea), .mant(ma));
    fp_unpack u_b (.x(b), .sign(sb), .exp(eb), .mant(mb));

    // get the x0 from look up table
    wire [31:0] x0;
    seed_lut u_lut (.idx(mb[22:20]), .x0(x0));

    // using the standard multiplyier that is the there for the project
    wire [31:0] t_w = 32'h8000_0000 - e_r; // 2 - e 
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
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    sign_r <= sa ^ sb;
                    ea_r   <= ea;
                    eb_r   <= eb;
                    ma_r   <= ma;
                    d_r    <= {mb, 7'b0};
                    x_r    <= x0;
                    iter   <= 3'd0;
                    state  <= (num_iter == 3'd0) ? S_Q : S_E;
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
                    done   <= 1'b1;
                    state  <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
