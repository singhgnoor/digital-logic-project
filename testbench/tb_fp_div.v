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

    fp_div dut (.clk(clk), 
                .rst(rst), 
                .start(start), 
                .a(a), 
                .b(b), 
                .num_iter(num_iter),
                .result(result), 
                .done(done), 
                .busy(busy), 
                .x_dbg(x_dbg), 
                .iter_dbg(iter_dbg));

    always #5 clk = ~clk;

    reg [95:0] vec [0:VECTOR_COUNT-1];
    integer n, nvec, verbose, iters, cycles, run_limit;
    integer n_exact, n_1ulp, n_bad, max_ulp, lat_first;
    integer err;
    reg [31:0] exp_q;
    reg [31:0] last_x;

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

        $display("\n\n\t\t\tFinal Results\n");
        $display("Vectors : %0d      NR iterations : %0d      latency : %0d cycles", nvec, iters, lat_first);
        $display("Exact   : %0d (%f %%)", n_exact, 100.0*n_exact/nvec);
        $display("1 ULP   : %0d (%f %%)", n_1ulp,  100.0*n_1ulp/nvec);
        $display(">1 ULP  : %0d     max ULP error : %0d", n_bad, max_ulp);
        $finish;
    end
endmodule
