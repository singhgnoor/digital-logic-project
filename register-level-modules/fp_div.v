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
// STAGE 4 CHANGE START: seed modes, convergence monitor, square root, fault injection
// STAGE 4: op = 1 -> square root of a, same registers reused:  d_r = x (radicand), x_r = y ~ 1/sqrt(x)
//   IDLE -> (S_Y2 -> S_XY2 -> S_YT) x num_iter -> S_SQ -> S_OUT -> IDLE
//   S_Y2 : e = y * y
//   S_XY2: e = x * (y*y)            (residual for the monitor)
//   S_YT : y = y * (3 - x*y*y) / 2  (the /2 is a one bit bigger shift)
//   S_SQ : root = x * y
//   S_OUT: exact rounding check (2q+1)^2 vs 4x on the shared multiplier, pack
// STAGE 4: auto_stop = 1 -> conv_monitor checks |1 - e| < 2^-15 in S_E / S_XY2, if yes this is the last iteration
// STAGE 4: S_OUT of a divide also computes d*x on the (free) multiplier -> fault_det if |1 - d*x| >= 2^-26
// STAGE 4 CHANGE END

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

STAGE 4: the module now ALSO does square root (op = 1), name kept as fp_div so nothing else breaks
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
    // STAGE 4 CHANGE START: operation select + convergence monitor enable
    input  wire op,        // 0 -> a / b,  1 -> sqrt(a)  (b is ignored)
    input  wire auto_stop, // 1 -> stop when the residual is small, num_iter is then the max
    // STAGE 4 CHANGE END
    output reg  [31:0] result,
    output reg done, // 1-cycle pulse, result valid
    output wire busy,
    // STAGE 3 CHANGE START: IEEE exception flags, valid together with 'done'
    output reg  [3:0]  flags, // {invalid, div_by_zero, overflow, underflow}
    // STAGE 3 CHANGE END
    // debug or analysis
    output wire [31:0] x_dbg, // current reciprocal estimate, for convergence monitoring
    // STAGE 4 CHANGE START: monitor outputs + residual fault check
    // STAGE 3 (old): output wire [2:0]  iter_dbg
    output wire [2:0]  iter_dbg,
    output wire [2:0]  iters_used, // iterations actually done, valid with done
    output reg  converged,         // 1 -> the monitor stopped the loop before num_iter
    output reg  fault_det          // 1 -> divide result failed the final |1 - d*x| < 2^-26 check
    // STAGE 4 CHANGE END
);
    // STAGE 4 CHANGE START: seed select + 4 bit state for the sqrt states
    // STAGE 3 (old): localparam IDLE = 3'd0, S_E = 3'd1, S_X = 3'd2, S_Q = 3'd3, S_OUT = 3'd4;
    // STAGE 3 (old): reg [2:0]  state;
    parameter SEED_MODE = 0; // 0 -> seed_lut (stage 3), 1 -> single line, 2 -> piecewise linear (8 segments)

    localparam IDLE = 4'd0, S_E = 4'd1, S_X = 4'd2, S_Q = 4'd3, S_OUT = 4'd4;
    localparam S_Y2 = 4'd5, S_XY2 = 4'd6, S_YT = 4'd7, S_SQ = 4'd8; // sqrt

    reg [3:0]  state;
    reg op_r;          // 1 -> current job is a sqrt
    reg conv_r;        // monitor result of this iteration
    reg [23:0] root_r; // truncated sqrt mantissa
    // STAGE 4 CHANGE END
    reg [2:0]  iter;
    reg sign_r;
    reg [7:0]  ea_r, eb_r;
    reg [23:0] ma_r;
    reg [31:0] d_r, x_r, e_r;
    reg [55:0] prod_r;

    assign busy = (state != IDLE);
    assign x_dbg = x_r;
    assign iter_dbg = iter;
    assign iters_used = iter; // STAGE 4

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

    // STAGE 4 CHANGE START: sqrt special cases (only A is used)
    wire sq_special = nana | a_zero | infa | sa;               // anything but a positive normal
    wire sq_invalid = (nana & ~a[22]) | (sa & ~a_zero & ~nana); // sNaN, sqrt(negative)
    wire [31:0] sq_special_val = nana   ? {a[31:23], 1'b1, a[21:0]} : // NaN propagated (quietened)
                                 a_zero ? {sa, 31'd0} :               // +-0 (subnormal -> DAZ)
                                 sa     ? 32'h7FC0_0000 :             // sqrt(negative) -> default NaN
                                          32'h7F80_0000;              // sqrt(+Inf) = +Inf
    // STAGE 4 CHANGE END

    // get the x0 from look up table
    wire [31:0] x0;
    // STAGE 4 CHANGE START: seed picked by SEED_MODE
    // STAGE 3 (old): seed_lut u_lut (.idx(mb[22:20]), .x0(x0));
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
    // STAGE 4 CHANGE END

    // using the standard multiplyier that is the there for the project
    // STAGE 3 CHANGE START: 2 - e through the gate-level subtractor
    // STAGE 2 (old): wire [31:0] t_w = 32'h8000_0000 - e_r; // 2 - e
    wire [31:0] t_w; // 2 - e
    // STAGE 4 CHANGE START: same subtractor makes 3 - x*y*y for sqrt
    // STAGE 3 (old): sub32 u_sub (.a(32'h8000_0000), .b(e_r), .diff(t_w), .borrow());
    sub32 u_sub (.a(op_r ? 32'hC000_0000 : 32'h8000_0000), .b(e_r), .diff(t_w), .borrow()); // 2 - e  or  3 - x*y*y
    // STAGE 4 CHANGE END
    // STAGE 3 CHANGE END
    reg  [31:0] mul_a, mul_b;
    wire [63:0] mul_p;
    wire [24:0] chk_w; // STAGE 4: 2q+1 for the sqrt rounding check

    // STAGE 4 CHANGE START: sqrt states, S_OUT uses the multiplier for the checks
    // STAGE 3 (old): S_E: d_r * x_r,   S_X: x_r * t_w,   S_Q: {8'b0, ma_r} * x_r
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
    // STAGE 4 CHANGE END
    mant_mul u_mul (.a(mul_a), .b(mul_b), .p(mul_p));

    // STAGE 4 CHANGE START: convergence monitor + loop exit
    wire conv_w, res_ok_w;
    conv_monitor u_mon (.e(mul_p[62:31]), .conv(conv_w), .res_ok(res_ok_w)); // e = d*x or x*y*y

    wire last_w  = ((iter + 3'd1) == num_iter);   // cap reached
    wire early_w = auto_stop & conv_r & ~last_w;  // monitor says this update is good enough
    // STAGE 4 CHANGE END

    // round to required bits and pack
    wire signed [9:0] exp_base = $signed({2'b00, ea_r}) - $signed({2'b00, eb_r}) + 10'sd127;
    wire [31:0] res_w;
    wire        ovf_w, unf_w;
    fp_round_pack u_rp (.sign(sign_r), .exp_base(exp_base), .prod(prod_r),
                        .result(res_w), .overflow(ovf_w), .underflow(unf_w));

    // STAGE 4 CHANGE START: sqrt exact rounding + pack
    wire [31:0] sq_res_w;
    sqrt_pack u_sp (.exp_a(ea_r), .x(d_r), .q(root_r), .sq_hi(mul_p[49:25]),
                    .chk(chk_w), .result(sq_res_w));
    // STAGE 4 CHANGE END

    // STAGE 4 CHANGE START: fault injection, simulation only (iverilog -DFAULT_INJ ...)
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
    // STAGE 4 CHANGE END

    // controlling the cycles [FSM (Finite state machine)]
    always @(posedge clk) begin
        if (rst) begin
            state  <= IDLE;
            done   <= 1'b0;
            result <= 32'd0;
            iter   <= 3'd0;
            flags  <= 4'd0;   // STAGE 3
            // STAGE 4 CHANGE START
            op_r      <= 1'b0;
            conv_r    <= 1'b0;
            converged <= 1'b0;
            fault_det <= 1'b0;
            // STAGE 4 CHANGE END
        end else begin
            done <= 1'b0;
            case (state)
                // STAGE 3 CHANGE START: special operands answered directly from IDLE
                // STAGE 2 (old): IDLE: if (start) begin <load regs, go to S_E / S_Q> end
                // STAGE 4 CHANGE START: sqrt start (op = 1), fault injection on the loaded registers
                // STAGE 3 (old): if (special) begin result <= special_val; flags <= {invalid_w, dbz_w, 2'b00}; ...
                // STAGE 3 (old): ma_r <= ma;  d_r <= {mb, 7'b0};  x_r <= x0;
                // STAGE 3 (old): state <= (num_iter == 3'd0) ? S_Q : S_E;
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
                // STAGE 4 CHANGE END
                // STAGE 3 CHANGE END
                // STAGE 4 CHANGE START: monitor in S_E, early exit in S_X, fault injection
                // STAGE 3 (old): S_E: e_r <= mul_p[62:31];
                // STAGE 3 (old): S_X: x_r <= mul_p[61:30];  state <= ((iter + 3'd1) == num_iter) ? S_Q : S_E;
                // STAGE 3 (old): S_Q: prod_r <= mul_p[55:0];
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
                    flags     <= op_r ? 4'd0 : {2'b00, ovf_w, unf_w};   // STAGE 3
                    fault_det <= ~op_r & ~res_ok_w;
                    done      <= 1'b1;
                    state     <= IDLE;
                end
                // STAGE 4 CHANGE END
                default: state <= IDLE;
            endcase
        end
    end
endmodule
