// STAGE 3 - NEW FILE
`timescale 1ns/1ps

/*
cla8 -> 8 bit carry look ahead adder, gate level
g = a & b (bit generates a carry),  p = a ^ b (bit passes the carry in to the next bit)
every carry is written directly from g, p and cin, so no carry waits for the previous one:
    c[i+1] = g[i] | p[i]&g[i-1] | p[i]&p[i-1]&g[i-2] | ... | p[i]&...&p[0]&cin
sum = p ^ c
building block of sub32 (4 of these, carry rippling between the blocks)
*/

module cla8 (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire cin,
    output wire [7:0] sum,
    output wire cout
);
    wire [7:0] g = a & b; // generate
    wire [7:0] p = a ^ b; // propagate
    wire [8:0] c;

    assign c[0] = cin;
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & g[0]) | (&p[1:0] & cin);
    assign c[3] = g[2] | (p[2] & g[1]) | (&p[2:1] & g[0]) | (&p[2:0] & cin);
    assign c[4] = g[3] | (p[3] & g[2]) | (&p[3:2] & g[1]) | (&p[3:1] & g[0]) | (&p[3:0] & cin);
    assign c[5] = g[4] | (p[4] & g[3]) | (&p[4:3] & g[2]) | (&p[4:2] & g[1]) | (&p[4:1] & g[0])
                | (&p[4:0] & cin);
    assign c[6] = g[5] | (p[5] & g[4]) | (&p[5:4] & g[3]) | (&p[5:3] & g[2]) | (&p[5:2] & g[1])
                | (&p[5:1] & g[0]) | (&p[5:0] & cin);
    assign c[7] = g[6] | (p[6] & g[5]) | (&p[6:5] & g[4]) | (&p[6:4] & g[3]) | (&p[6:3] & g[2])
                | (&p[6:2] & g[1]) | (&p[6:1] & g[0]) | (&p[6:0] & cin);
    assign c[8] = g[7] | (p[7] & g[6]) | (&p[7:6] & g[5]) | (&p[7:5] & g[4]) | (&p[7:4] & g[3])
                | (&p[7:3] & g[2]) | (&p[7:2] & g[1]) | (&p[7:1] & g[0]) | (&p[7:0] & cin);

    assign sum  = p ^ c[7:0];
    assign cout = c[8];
endmodule
