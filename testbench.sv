// combined testbench

`timescale 1ns/1ps
// Error is measured as ULP distance between given and the expected result.
//   +ITER=n      number of NR iterations (max when +AUTO=1)          default 3
//   +AUTO=0/1    convergence monitor off / on                        default 0
//   +OP=0/1      0 -> divide (vectors.hex), 1 -> sqrt (vectors_sqrt.hex, or +SQRT_VEC=<file>)
//   +CSV=1       write results.csv (every vector) and conv_log.csv (every NR step of the first VERBOSE vectors)
//   seed mode is a parameter (fixed hardware, not a run time choice):  iverilog -Ptb_fp_div.SEED_MODE=2 ...
module tb_fp_div;
    localparam integer VECTOR_COUNT = 4000;
    parameter  integer MAX_RUN = 20; // number of iterations to run
    localparam integer SQRT_COUNT = 2000;
    parameter  integer SEED_MODE = 0; // 0 = LUT, 1 = line, 2 = piecewise linear

    reg clk = 0, rst = 1, start = 0;
    reg  [31:0] a, b;
    reg  [2:0]  num_iter;
    wire [31:0] result, x_dbg;
    wire done, busy;
    wire [2:0]  iter_dbg;
    wire [3:0]  flags;
    reg  op = 0, auto_stop = 0;
    wire [2:0] iters_used;
    wire converged, fault_det;

    fp_div #(.SEED_MODE(SEED_MODE)) dut (.clk(clk),
                .rst(rst),
                .start(start),
                .a(a),
                .b(b),
                .num_iter(num_iter),
                .op(op),
                .auto_stop(auto_stop),
                .result(result),
                .done(done),
                .busy(busy),
                .flags(flags),
                .x_dbg(x_dbg),
                .iter_dbg(iter_dbg),
                .iters_used(iters_used),
                .converged(converged),
                .fault_det(fault_det));

    always #5 clk = ~clk;

    reg [95:0] vec [0:VECTOR_COUNT-1];
    integer n, nvec, verbose, iters, cycles, run_limit;
    integer n_exact, n_1ulp, n_bad, max_ulp, lat_first;
    integer err;
    reg [31:0] exp_q;
    reg [31:0] last_x;

    reg [63:0] svec [0:SQRT_COUNT-1];
    reg [8*64:1] sqrt_file;
    integer ops, autos, csv, f_res, f_conv;
    integer hist [0:7];
    integer lat_min, lat_max, lat_sum, n_conv, n_fdet;
    real xr, dr, resid;

    localparam integer NSPEC = 25;
    reg [99:0] spec [0:NSPEC-1];
    integer k, n_spec_fail;
    reg [3:0] exp_f;
    initial begin
        //            A             B             expected      flags (I D O U)
        spec[ 0] = {32'h0000_0000, 32'h0000_0000, 32'h7FC0_0000, 4'b1000}; //  0 / 0       -> NaN, invalid
        spec[ 1] = {32'h8000_0000, 32'h0000_0000, 32'h7FC0_0000, 4'b1000}; // -0 / 0       -> NaN, invalid
        spec[ 2] = {32'h7F80_0000, 32'hFF80_0000, 32'h7FC0_0000, 4'b1000}; // Inf / -Inf   -> NaN, invalid
        spec[ 3] = {32'h3F80_0000, 32'h0000_0000, 32'h7F80_0000, 4'b0100}; //  1 / 0       -> +Inf, div by zero
        spec[ 4] = {32'hBF80_0000, 32'h0000_0000, 32'hFF80_0000, 4'b0100}; // -1 / 0       -> -Inf
        spec[ 5] = {32'h3F80_0000, 32'h8000_0000, 32'hFF80_0000, 4'b0100}; //  1 / -0      -> -Inf
        spec[ 6] = {32'h0000_0000, 32'h3F00_0000, 32'h0000_0000, 4'b0000}; //  0 / 0.5     -> +0
        spec[ 7] = {32'h8000_0000, 32'h4040_0000, 32'h8000_0000, 4'b0000}; // -0 / 3       -> -0
        spec[ 8] = {32'h7F80_0000, 32'h4000_0000, 32'h7F80_0000, 4'b0000}; // Inf / 2      -> +Inf
        spec[ 9] = {32'hFF80_0000, 32'h4000_0000, 32'hFF80_0000, 4'b0000}; // -Inf / 2     -> -Inf
        spec[10] = {32'h7F80_0000, 32'h0000_0000, 32'h7F80_0000, 4'b0000}; // Inf / 0      -> +Inf (no dbz)
        spec[11] = {32'h3F80_0000, 32'h7F80_0000, 32'h0000_0000, 4'b0000}; //  1 / Inf     -> +0
        spec[12] = {32'hC0A0_0000, 32'h7F80_0000, 32'h8000_0000, 4'b0000}; // -5 / Inf     -> -0
        spec[13] = {32'h0000_0000, 32'h7F80_0000, 32'h0000_0000, 4'b0000}; //  0 / Inf     -> +0
        spec[14] = {32'h7FC1_2345, 32'h3F80_0000, 32'h7FC1_2345, 4'b0000}; // qNaN / 1     -> same qNaN
        spec[15] = {32'h3F80_0000, 32'hFFC0_0001, 32'hFFC0_0001, 4'b0000}; // 1 / qNaN     -> same qNaN
        spec[16] = {32'h7F80_0001, 32'h3F80_0000, 32'h7FC0_0001, 4'b1000}; // sNaN / 1     -> quietened, invalid
        spec[17] = {32'h7FC0_0002, 32'h7F80_0003, 32'h7FC0_0002, 4'b1000}; // qNaN / sNaN  -> A's NaN, invalid
        spec[18] = {32'h7FC0_0004, 32'h0000_0000, 32'h7FC0_0004, 4'b0000}; // qNaN / 0     -> NaN, no dbz
        spec[19] = {32'h0000_0001, 32'h3F80_0000, 32'h0000_0000, 4'b0000}; // sub / 1      -> +0  (DAZ)
        spec[20] = {32'h3F80_0000, 32'h0040_0000, 32'h7F80_0000, 4'b0100}; // 1 / sub      -> +Inf (DAZ)
        spec[21] = {32'h0000_0001, 32'h8040_0000, 32'h7FC0_0000, 4'b1000}; // sub / -sub   -> NaN  (DAZ)
        spec[22] = {32'h8040_0000, 32'h4000_0000, 32'h8000_0000, 4'b0000}; // -sub / 2     -> -0   (DAZ)
        spec[23] = {32'h7149_F2CA, 32'h0DA2_4260, 32'h7F80_0000, 4'b0010}; // 1e30 / 1e-30 -> +Inf, overflow
        spec[24] = {32'h0DA2_4260, 32'h7149_F2CA, 32'h0000_0000, 4'b0001}; // 1e-30 / 1e30 -> +0, underflow (FTZ)
    end

    localparam integer NSQS = 14;
    reg [67:0] sqs [0:NSQS-1];
    initial begin
        //           A              expected       flags (I D O U)
        sqs[ 0] = {32'h0000_0000, 32'h0000_0000, 4'b0000}; // sqrt(+0)    -> +0
        sqs[ 1] = {32'h8000_0000, 32'h8000_0000, 4'b0000}; // sqrt(-0)    -> -0
        sqs[ 2] = {32'h7F80_0000, 32'h7F80_0000, 4'b0000}; // sqrt(+Inf)  -> +Inf
        sqs[ 3] = {32'hFF80_0000, 32'h7FC0_0000, 4'b1000}; // sqrt(-Inf)  -> NaN, invalid
        sqs[ 4] = {32'hBF80_0000, 32'h7FC0_0000, 4'b1000}; // sqrt(-1)    -> NaN, invalid
        sqs[ 5] = {32'h7FC1_2345, 32'h7FC1_2345, 4'b0000}; // qNaN        -> same qNaN
        sqs[ 6] = {32'hFFC0_0001, 32'hFFC0_0001, 4'b0000}; // -qNaN       -> same NaN, no invalid
        sqs[ 7] = {32'h7F80_0001, 32'h7FC0_0001, 4'b1000}; // sNaN        -> quietened, invalid
        sqs[ 8] = {32'h0000_0001, 32'h0000_0000, 4'b0000}; // sub         -> +0 (DAZ)
        sqs[ 9] = {32'h8040_0000, 32'h8000_0000, 4'b0000}; // -sub        -> -0 (DAZ, not NaN)
        sqs[10] = {32'h4080_0000, 32'h4000_0000, 4'b0000}; // sqrt(4)     -> 2
        sqs[11] = {32'h3F7F_FFFF, 32'h3F7F_FFFF, 4'b0000}; // near tie, exact rounding needed
        sqs[12] = {32'h407F_FFFF, 32'h3FFF_FFFF, 4'b0000}; // near tie
        sqs[13] = {32'h7F7F_FFFF, 32'h5F7F_FFFF, 4'b0000}; // max float
    end

    // print the guess after every NR step (convergence monitoring)
    always @(posedge clk)
        if (busy && n < verbose && x_dbg !== last_x) begin
            $display("      x[%0d] = %h  (%f)", iter_dbg, x_dbg, x_dbg / 1073741824.0);
            last_x = x_dbg;
            if (csv) begin
                xr = x_dbg / 1073741824.0;
                if (ops == 0) begin
                    dr    = {1'b1, b[22:0]} / 16777216.0;            // d = mant_B / 2
                    resid = 1.0 - dr * xr;
                end else begin
                    dr    = a[23] ? {1'b1, a[22:0]} / 33554432.0       // ea odd  -> x = m/4
                                  : {1'b1, a[22:0]} / 16777216.0;      // ea even -> x = m/2
                    resid = 1.0 - dr * xr * xr;
                end
                if (resid < 0) resid = -resid;
                $fdisplay(f_conv, "%0d,%0d,%h,%.10f,%e", n, iter_dbg, x_dbg, xr, resid);
            end
        end

    initial begin
        if (!$value$plusargs("ITER=%d", iters))       iters = 3;
        if (!$value$plusargs("VERBOSE=%d", verbose))   verbose = 10;
        if (!$value$plusargs("RUN=%d", run_limit))     run_limit = MAX_RUN;
        if (!$value$plusargs("AUTO=%d", autos))         autos = 0;
        if (!$value$plusargs("OP=%d", ops))             ops = 0;
        if (!$value$plusargs("CSV=%d", csv))            csv = 0;
        if (!$value$plusargs("SQRT_VEC=%s", sqrt_file)) sqrt_file = "vectors_sqrt.hex";
        num_iter = iters;
        op = (ops != 0);
        auto_stop = (autos != 0);
        nvec = 0;
        if (ops == 0) begin
            $readmemh("vectors.hex", vec);
            while (nvec < VECTOR_COUNT && vec[nvec] !== 96'bx) nvec = nvec + 1;
        end else begin
            $readmemh(sqrt_file, svec);
            while (nvec < SQRT_COUNT && svec[nvec] !== 64'bx) nvec = nvec + 1;
        end
        if (run_limit > 0 && run_limit < nvec) nvec = run_limit;
        if (csv) begin
            f_res  = $fopen("results.csv", "w");
            f_conv = $fopen("conv_log.csv", "w");
            $fdisplay(f_res,  "vec,a,b,result,expected,ulp,cycles,iters_used");
            $fdisplay(f_conv, "vec,iter,x_hex,x_dec,residual");
        end
        $display("Testbench ready for %0d vectors, Number of NR iterations = %0d", nvec, iters);
        $display("Operation = %s   seed mode = %0d   auto_stop = %0d", ops ? "sqrt" : "divide", SEED_MODE, autos);
        for (k = 0; k < 8; k = k + 1) hist[k] = 0;
        lat_min = 1000; lat_max = 0; lat_sum = 0; n_conv = 0; n_fdet = 0;

        n_exact = 0; n_1ulp = 0; n_bad = 0; max_ulp = 0; lat_first = 0;
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst = 0;

        for (n = 0; n < nvec; n = n + 1) begin
            @(negedge clk);
            if (ops == 0) begin
                a = vec[n][95:64];  b = vec[n][63:32];  exp_q = vec[n][31:0];
            end else begin
                a = svec[n][63:32]; b = 32'd0;          exp_q = svec[n][31:0];
            end
            last_x = 32'hx;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            cycles = 1;
            while (!done && cycles < 100) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (!done) begin // failsafe for hangup
                $display("ERROR!!! Timeout on vector %0d! DUT hung.", n);
                $finish;
            end
            if (n == 0) lat_first = cycles;

            // Error calculation for each example
            if (result[31] != exp_q[31]) err = 1000000;
            else err = (result[30:0] > exp_q[30:0]) ? result[30:0] - exp_q[30:0]
                                                     : exp_q[30:0] - result[30:0];
            if (err == 0) n_exact = n_exact + 1;
            else if (err == 1) n_1ulp = n_1ulp + 1;
            else n_bad = n_bad + 1;
            if (err > max_ulp) max_ulp = err;

            hist[iters_used] = hist[iters_used] + 1;
            lat_sum = lat_sum + cycles;
            if (cycles < lat_min) lat_min = cycles;
            if (cycles > lat_max) lat_max = cycles;
            if (converged) n_conv = n_conv + 1;
            if (fault_det) n_fdet = n_fdet + 1;
            if (csv) $fdisplay(f_res, "%0d,%h,%h,%h,%h,%0d,%0d,%0d", n, a, b, result, exp_q, err, cycles, iters_used);

            if (n < verbose || err > 1)
                $display("[%0d] A=%h B=%h  hw=%h  exp=%h  ULP err=%0d  cycles=%0d %s",
                         n, a, b, result, exp_q, err, cycles, (err > 1) ? "<-- MISMATCH" : "");
        end

        $display("\n\t\t\tSpecial cases\n");
        n_spec_fail = 0;
        for (k = 0; k < (ops ? NSQS : NSPEC); k = k + 1) begin
            @(negedge clk);
            if (ops == 0) begin
                a = spec[k][99:68];  b = spec[k][67:36];  exp_q = spec[k][35:4];  exp_f = spec[k][3:0];
            end else begin
                a = sqs[k][67:36];   b = 32'd0;           exp_q = sqs[k][35:4];   exp_f = sqs[k][3:0];
            end
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            cycles = 1;
            while (!done && cycles < 100) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            if (!done) begin
                $display("ERROR!!! Timeout on special vector %0d! DUT hung.", k);
                $finish;
            end
            if (result !== exp_q || flags !== exp_f) n_spec_fail = n_spec_fail + 1;
            $display("[S%0d] A=%h B=%h  hw=%h  exp=%h  flags=%b exp_flags=%b  cycles=%0d %s",
                     k, a, b, result, exp_q, flags, exp_f, cycles,
                     (result !== exp_q || flags !== exp_f) ? "<-- MISMATCH" : "ok");
        end

        $display("\n\n\t\t\tFinal Results\n");
        $display("Vectors : %0d      NR iterations : %0d      latency : %0d cycles", nvec, iters, lat_first);
        $display("Exact   : %0d (%f %%)", n_exact, 100.0*n_exact/nvec);
        $display("1 ULP   : %0d (%f %%)", n_1ulp,  100.0*n_1ulp/nvec);
        $display(">1 ULP  : %0d     max ULP error : %0d", n_bad, max_ulp);
        $display("Special : %0d / %0d passed", (ops ? NSQS : NSPEC) - n_spec_fail, ops ? NSQS : NSPEC);
        $display("Iterations used : 0:%0d 1:%0d 2:%0d 3:%0d 4:%0d 5:%0d 6:%0d 7:%0d   (average %f)",
                 hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7],
                 (1.0*hist[1] + 2*hist[2] + 3*hist[3] + 4*hist[4] + 5*hist[5] + 6*hist[6] + 7*hist[7]) / nvec);
        $display("Latency : min %0d   average %f   max %0d cycles", lat_min, 1.0*lat_sum/nvec, lat_max);
        $display("Monitor stopped early : %0d     fault_det raised : %0d", n_conv, n_fdet);
        if (csv) begin
            $fclose(f_res);
            $fclose(f_conv);
        end
        $finish;
    end
endmodule
