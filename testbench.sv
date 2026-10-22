// combined testbench

`timescale 1ns/1ps
// Error is measured as ULP distance between given and the expected result.
module tb_fp_div;
    localparam integer VECTOR_COUNT = 4000;
    parameter  integer MAX_RUN = 20; // number of iterations to run

    reg clk = 0, rst = 1, start = 0;
    reg  [31:0] a, b;
    reg  [2:0]  num_iter;
    wire [31:0] result, x_dbg;
    wire done, busy;
    wire [2:0]  iter_dbg;
    wire [3:0]  flags;   // STAGE 3: {invalid, div_by_zero, overflow, underflow}

    fp_div dut (.clk(clk), 
                .rst(rst), 
                .start(start), 
                .a(a), 
                .b(b), 
                .num_iter(num_iter),
                .result(result), 
                .done(done), 
                .busy(busy), 
                .flags(flags),
                .x_dbg(x_dbg), 
                .iter_dbg(iter_dbg));

    always #5 clk = ~clk;

    reg [95:0] vec [0:VECTOR_COUNT-1];
    integer n, nvec, verbose, iters, cycles, run_limit;
    integer n_exact, n_1ulp, n_bad, max_ulp, lat_first;
    integer err;
    reg [31:0] exp_q;
    reg [31:0] last_x;

    // STAGE 3 CHANGE START: directed special-case vectors {A, B, expected result, expected flags}
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
    // STAGE 3 CHANGE END

    // print the guess after every NR step (convergence monitoring)
    always @(posedge clk)
        if (busy && n < verbose && x_dbg !== last_x) begin
            $display("      x[%0d] = %h  (%f)", iter_dbg, x_dbg, x_dbg / 1073741824.0);
            last_x = x_dbg;
        end

    initial begin
        if (!$value$plusargs("ITER=%d", iters))       iters = 3;
        if (!$value$plusargs("VERBOSE=%d", verbose))   verbose = 10;
        if (!$value$plusargs("RUN=%d", run_limit))     run_limit = MAX_RUN;
        num_iter = iters;
        $readmemh("vectors.hex", vec);
        nvec = 0;
        while (nvec < VECTOR_COUNT && vec[nvec] !== 96'bx) nvec = nvec + 1;
        if (run_limit > 0 && run_limit < nvec) nvec = run_limit;
        $display("Testbench ready for %0d vectors, Number of NR iterations = %0d", nvec, iters);

        n_exact = 0; n_1ulp = 0; n_bad = 0; max_ulp = 0; lat_first = 0;
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst = 0;

        for (n = 0; n < nvec; n = n + 1) begin
            @(negedge clk);
            a = vec[n][95:64];  b = vec[n][63:32];  exp_q = vec[n][31:0];
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

            if (n < verbose || err > 1)
                $display("[%0d] A=%h B=%h  hw=%h  exp=%h  ULP err=%0d  cycles=%0d %s",
                         n, a, b, result, exp_q, err, cycles, (err > 1) ? "<-- MISMATCH" : "");
        end

        // STAGE 3 CHANGE START: special-case section (exact bit match + flags + latency)
        $display("\n\t\t\tSpecial cases\n");
        n_spec_fail = 0;
        for (k = 0; k < NSPEC; k = k + 1) begin
            @(negedge clk);
            a = spec[k][99:68];  b = spec[k][67:36];  exp_q = spec[k][35:4];  exp_f = spec[k][3:0];
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
        // STAGE 3 CHANGE END

        $display("\n\n\t\t\tFinal Results\n");
        $display("Vectors : %0d      NR iterations : %0d      latency : %0d cycles", nvec, iters, lat_first);
        $display("Exact   : %0d (%f %%)", n_exact, 100.0*n_exact/nvec);
        $display("1 ULP   : %0d (%f %%)", n_1ulp,  100.0*n_1ulp/nvec);
        $display(">1 ULP  : %0d     max ULP error : %0d", n_bad, max_ulp);
        $display("Special : %0d / %0d passed", NSPEC - n_spec_fail, NSPEC);   // STAGE 3
        $finish;
    end
endmodule
