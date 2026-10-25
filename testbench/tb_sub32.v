`timescale 1ns/1ps
// Checks sub32 against the behavioural '-' : diff = a - b, borrow = (a < b)
module tb_sub32;
    parameter integer N_RAND = 20000;

    reg  [31:0] a, b;
    wire [31:0] diff;
    wire borrow;

    sub32 dut (.a(a), .b(b), .diff(diff), .borrow(borrow));

    integer i, seed, n, n_bad;

    task check;
        input [31:0] x, y;
        begin
            a = x; b = y;
            #1;
            n = n + 1;
            if (diff !== a - b || borrow !== (a < b)) begin
                n_bad = n_bad + 1;
                if (n_bad <= 5) $display("MISMATCH  %h - %h  diff=%h borrow=%b  exp=%h borrow=%b",
                                         a, b, diff, borrow, a - b, (a < b));
            end
        end
    endtask

    initial begin
        seed = 14;
        n = 0; n_bad = 0;

        // corner cases
        check(32'h00000000, 32'h00000000);  check(32'h00000000, 32'hFFFFFFFF);
        check(32'hFFFFFFFF, 32'h00000000);  check(32'hFFFFFFFF, 32'hFFFFFFFF);
        check(32'h80000000, 32'h00000000);  check(32'h00000000, 32'h80000000);
        check(32'h80000000, 32'h80000000);  check(32'h80000000, 32'h00000001);
        check(32'h00000001, 32'h00000002);  check(32'h12345678, 32'h12345678); // a<b, a==b
        check(32'h80000000, 32'h40000000);  check(32'h80000000, 32'h3FFFFFFF); // 2 - e with e ~ 1.0

        // random
        for (i = 0; i < N_RAND; i = i + 1)
            check($random(seed), $random(seed));

        $display("\n\n\t\t\tFinal Results\n");
        $display("sub32 : %0d cases   mismatches : %0d", n, n_bad);
        if (n_bad == 0) $display("ALL PASS");
        else $display("FAIL");
        $finish;
    end
endmodule
