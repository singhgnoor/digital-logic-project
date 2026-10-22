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
    // STAGE 3 CHANGE START: IEEE exception flags, valid together with 'done'
    output reg  [3:0]  flags, // {invalid, div_by_zero, overflow, underflow}
    // STAGE 3 CHANGE END
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
    // STAGE 3 CHANGE START: operand classification + special-case result
    // STAGE 2 (old): fp_unpack u_a (.x(a), .sign(sa), .exp(ea), .mant(ma));
    // STAGE 2 (old): fp_unpack u_b (.x(b), .sign(sb), .exp(eb), .mant(mb));
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
    // STAGE 3 CHANGE END

    // get the x0 from look up table
    wire [31:0] x0;
    seed_lut u_lut (.idx(mb[22:20]), .x0(x0));

    // using the standard multiplyier that is the there for the project
    // STAGE 3 CHANGE START: 2 - e through the gate-level subtractor
    // STAGE 2 (old): wire [31:0] t_w = 32'h8000_0000 - e_r; // 2 - e
    wire [31:0] t_w; // 2 - e
    sub32 u_sub (.a(32'h8000_0000), .b(e_r), .diff(t_w), .borrow());
    // STAGE 3 CHANGE END
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
                // STAGE 3 CHANGE START: special operands answered directly from IDLE
                // STAGE 2 (old): IDLE: if (start) begin <load regs, go to S_E / S_Q> end
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
                // STAGE 3 CHANGE END
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
