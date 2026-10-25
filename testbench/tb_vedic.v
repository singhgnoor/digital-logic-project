`timescale 1ns/1ps
// Checks the Vedic multipliers against the behavioural '*'.
//   vedic_8x8   : exhaustive (all 65,536 pairs)
//   vedic_24x24 : corner cases + N_RAND random pairs
//   vedic_32x32 : corner cases + N_RAND random pairs
module tb_vedic;
    parameter integer N_RAND = 20000;

    reg  [7:0]  a8, b8;
    reg  [23:0] a24, b24;
    reg  [31:0] a32, b32;
    wire [15:0] p8;
    wire [47:0] p24;
    wire [63:0] p32;

    vedic_8x8   u_v8  (.a(a8),  .b(b8),  .p(p8));
    vedic_24x24 u_v24 (.a(a24), .b(b24), .p(p24));
    vedic_32x32 u_v32 (.a(a32), .b(b32), .p(p32));

    integer i, j, seed;
    integer n8, n24, n32, bad8, bad24, bad32;
    reg [63:0] exp_p; // expected product from '*'

    // apply one pair to the 24x24 and 32x32 and compare with '*'
    task check24;
        input [23:0] x, y;
        begin
            a24 = x; b24 = y;
            exp_p = a24 * b24;
            #1;
            n24 = n24 + 1;
            if (p24 !== exp_p[47:0]) begin
                bad24 = bad24 + 1;
                if (bad24 <= 5) $display("24x24 MISMATCH  %h * %h  vedic=%h  exp=%h", a24, b24, p24, exp_p[47:0]);
            end
        end
    endtask

    task check32;
        input [31:0] x, y;
        begin
            a32 = x; b32 = y;
            exp_p = a32 * b32;
            #1;
            n32 = n32 + 1;
            if (p32 !== exp_p[63:0]) begin
                bad32 = bad32 + 1;
                if (bad32 <= 5) $display("32x32 MISMATCH  %h * %h  vedic=%h  exp=%h", a32, b32, p32, exp_p[63:0]);
            end
        end
    endtask

    initial begin
        seed = 14;
        n8 = 0; n24 = 0; n32 = 0; bad8 = 0; bad24 = 0; bad32 = 0;

        // 8x8 exhaustive
        for (i = 0; i < 256; i = i + 1)
            for (j = 0; j < 256; j = j + 1) begin
                a8 = i; b8 = j;
                exp_p = a8 * b8;
                #1;
                n8 = n8 + 1;
                if (p8 !== exp_p[15:0]) begin
                    bad8 = bad8 + 1;
                    if (bad8 <= 5) $display("8x8 MISMATCH  %h * %h  vedic=%h  exp=%h", a8, b8, p8, exp_p[15:0]);
                end
            end

        // corner cases (zero, all ones, single bits, mantissa example 19 x 7)
        check24(24'h000000, 24'hFFFFFF);  check24(24'hFFFFFF, 24'hFFFFFF);
        check24(24'h800000, 24'h800000);  check24(24'h980000, 24'hE00000);
        check32(32'h00000000, 32'hFFFFFFFF);  check32(32'hFFFFFFFF, 32'hFFFFFFFF);
        check32(32'h80000000, 32'h80000000);  check32(32'h7FFFFFFF, 32'h80000001);

        // random
        for (i = 0; i < N_RAND; i = i + 1) begin
            check24($random(seed), $random(seed));
            check32($random(seed), $random(seed));
        end

        $display("\n\n\t\t\tFinal Results\n");
        $display("vedic_8x8   : %0d cases   mismatches : %0d", n8,  bad8);
        $display("vedic_24x24 : %0d cases   mismatches : %0d", n24, bad24);
        $display("vedic_32x32 : %0d cases   mismatches : %0d", n32, bad32);
        if (bad8 == 0 && bad24 == 0 && bad32 == 0) $display("ALL PASS");
        else $display("FAIL");
        $finish;
    end
endmodule
