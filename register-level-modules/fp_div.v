// STAGE 3 - MODIFIED (see markers below)
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
