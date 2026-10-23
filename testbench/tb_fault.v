// STAGE 4 - NEW FILE
`timescale 1ns/1ps
// Fault injection campaign for fp_div (simulation only), needs -DFAULT_INJ:
//   iverilog -g2005 -DFAULT_INJ -o fault.out ../register-level-modules/*.v ../testbench/tb_fault.v && vvp fault.out
// For every (site, iteration, bit) the first NVEC vectors of vectors.hex are run with one bit flipped.
// golden = the hardware's OWN fault free result (not the IEEE value), LUT seed, 3 iterations, auto_stop = 0
//   masked : 0 ULP from golden     ulp1 : 1 ULP     sdc : > 1 ULP (silent data corruption)
//   due    : no 'done' within 100 cycles (detected unrecoverable error, timeout)
//   detected : fault_det = 1 (final residual |1 - d*x| >= 2^-26)
// Writes fault_results.csv (same format as  golden_model --fault-subset, so the two can be diffed).
// The big campaign (all bits, 2000 vectors) is in the C++ model -> fault_campaign.csv
module tb_fault;
    localparam integer VECTOR_COUNT = 4000;
    parameter  integer NVEC = 200;

    reg clk = 0, rst = 1, start = 0;
    reg  [31:0] a, b;
    wire [31:0] result, x_dbg;
    wire done, busy, converged, fault_det;
    wire [2:0] iter_dbg, iters_used;
    wire [3:0] flags;

    fp_div dut (.clk(clk), .rst(rst), .start(start), .a(a), .b(b), .num_iter(3'd3),
                .op(1'b0), .auto_stop(1'b0),
                .result(result), .done(done), .busy(busy), .flags(flags),
                .x_dbg(x_dbg), .iter_dbg(iter_dbg),
                .iters_used(iters_used), .converged(converged), .fault_det(fault_det));
    // behavioural '*' instead of the gate level vedic: bit identical (tb_vedic), but ~10x faster to simulate
    defparam dut.u_mul.USE_VEDIC = 0;

    always #5 clk = ~clk;

`ifdef FAULT_INJ
    reg [95:0] vec [0:VECTOR_COUNT-1];
    reg [31:0] golden [0:NVEC-1];
    integer n, s, k, j, nk, nb, bit_pos, cycles, timed_out, err, f;
    integer masked, ulp1, sdc, due, det, sdc_det, false_alarm;
    integer tot_sdc [0:4];
    integer tot_det [0:4];

    // bits tried per site (the full sweep is done in the model)
    integer bits32 [0:7];
    integer bitsA  [0:5];
    integer bitsP  [0:7];
    initial begin
        bits32[0] = 0;  bits32[1] = 5;  bits32[2] = 10; bits32[3] = 15;
        bits32[4] = 20; bits32[5] = 25; bits32[6] = 28; bits32[7] = 31;
        bitsA[0]  = 0;  bitsA[1]  = 5;  bitsA[2]  = 10; bitsA[3]  = 15; bitsA[4] = 20; bitsA[5] = 23;
        bitsP[0]  = 0;  bitsP[1]  = 10; bitsP[2]  = 20; bitsP[3]  = 30;
        bitsP[4]  = 40; bitsP[5]  = 50; bitsP[6]  = 54; bitsP[7]  = 55;
    end

    function [8*6:1] site_name;
        input integer sel;
        case (sel)
            0: site_name = "x";
            1: site_name = "e";
            2: site_name = "d";
            3: site_name = "mant_A";
            default: site_name = "prod";
        endcase
    endfunction

    // one divide, cycles counted like tb_fp_div, timeout -> timed_out = 1
    task run_one;
        input [31:0] x, y;
        begin
            @(negedge clk);
            a = x;  b = y;
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            cycles = 1;
            while (!done && cycles < 100) begin
                @(posedge clk);
                cycles = cycles + 1;
            end
            timed_out = !done;
            if (timed_out) begin // get the FSM back to IDLE
                rst = 1; @(negedge clk); rst = 0;
            end
        end
    endtask

    initial begin
        $readmemh("vectors.hex", vec);
        f = $fopen("fault_results.csv", "w");
        $fdisplay(f, "site,iter,bit,n,masked,ulp1,sdc,due,detected,sdc_detected");
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst = 0;

        // fault free run -> golden results, and the false alarm count of the residual check
        dut.fi_en = 1'b0;
        false_alarm = 0;
        for (n = 0; n < NVEC; n = n + 1) begin
            run_one(vec[n][95:64], vec[n][63:32]);
            golden[n] = result;
            if (fault_det) false_alarm = false_alarm + 1;
        end
        $display("Fault free run : %0d vectors, fault_det false alarms = %0d", NVEC, false_alarm);
        $display("\nsite   iter bit |  masked  ulp1   sdc   due | detected  sdc_detected");

        for (s = 0; s < 5; s = s + 1) begin
            tot_sdc[s] = 0;  tot_det[s] = 0;
            nk = (s == 0) ? 4 : (s == 1) ? 3 : 1;   // x0..x3, e0..e2, the rest are written once
            nb = (s == 3) ? 6 : 8;
            for (k = 0; k < nk; k = k + 1)
                for (j = 0; j < nb; j = j + 1) begin
                    bit_pos = (s == 3) ? bitsA[j] : (s == 4) ? bitsP[j] : bits32[j];
                    dut.fi_sel  = s;
                    dut.fi_iter = k;
                    dut.fi_bit  = bit_pos;
                    masked = 0; ulp1 = 0; sdc = 0; due = 0; det = 0; sdc_det = 0;
                    for (n = 0; n < NVEC; n = n + 1) begin
                        dut.fi_en = 1'b1;
                        run_one(vec[n][95:64], vec[n][63:32]);
                        dut.fi_en = 1'b0;
                        if (timed_out) due = due + 1;
                        else begin
                            if (result[31] != golden[n][31]) err = 1000000;
                            else err = (result[30:0] > golden[n][30:0]) ? result[30:0] - golden[n][30:0]
                                                                         : golden[n][30:0] - result[30:0];
                            if (err == 0) masked = masked + 1;
                            else if (err == 1) ulp1 = ulp1 + 1;
                            else begin
                                sdc = sdc + 1;
                                if (fault_det) sdc_det = sdc_det + 1;
                            end
                            if (fault_det) det = det + 1;
                        end
                    end
                    tot_sdc[s] = tot_sdc[s] + sdc;
                    tot_det[s] = tot_det[s] + sdc_det;
                    $fdisplay(f, "%0s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                              site_name(s), k, bit_pos, NVEC, masked, ulp1, sdc, due, det, sdc_det);
                    $display("%6s  %0d  %2d  |  %5d %5d %5d %5d |  %5d  %5d",
                             site_name(s), k, bit_pos, masked, ulp1, sdc, due, det, sdc_det);
                end
        end
        $fclose(f);

        $display("\n\n\t\t\tFinal Results\n");
        for (s = 0; s < 5; s = s + 1)
            $display("%6s : %5d SDC runs, %5d detected by the residual check (%f %%)", site_name(s),
                     tot_sdc[s], tot_det[s], (tot_sdc[s] > 0) ? 100.0*tot_det[s]/tot_sdc[s] : 0.0);
        $display("fault free false alarms : %0d", false_alarm);
        $display("wrote fault_results.csv");
        $finish;
    end
`else
    initial begin
        $display("tb_fault needs the fault injection hooks: compile with  iverilog -DFAULT_INJ ...");
        $finish;
    end
`endif
endmodule
