# fp_div – IEEE-754 single-precision Newton-Raphson divider + square root (Stage 4)

register-level-modules/mant_mul.v       shared 32x32 multiplier wrapper, uses vedic_32x32 (USE_VEDIC=1 default, 0 = plain '*')
register-level-modules/vedic_mul.v      vedic multiplier, 2x2 gate level up to 32x32, plus 24x24 for mantissa
register-level-modules/full_adder.v     1 bit full adder
register-level-modules/cla8.v           8 bit carry lookahead adder
register-level-modules/sub32.v          32 bit subtractor, 4 cla8 blocks with ripple carry between them
register-level-modules/fp_unpack.v      sign / exponent / 24-bit mantissa + zero/sub/inf/nan classification   (Divya)
register-level-modules/seed_lut.v       8-entry reciprocal seed, Q2.30               (Gurnoor)
register-level-modules/seed_linear.v    STAGE 4: line / 8 segment piecewise linear seed, vedic_16x16 + sub32   (Gurnoor)
register-level-modules/rsqrt_lut.v      STAGE 4: 16-entry 1/sqrt seed, index = {exponent parity, 3 fraction bits}   (<owner>)
register-level-modules/conv_monitor.v   STAGE 4: residual |1 - e| check (convergence monitor + fault check)   (<owner>)
register-level-modules/sqrt_pack.v      STAGE 4: exact rounding check + pack for sqrt   (<owner>)
register-level-modules/fp_round_pack.v  normalise + round-nearest-even + pack        (Mansi)
register-level-modules/fp_div.v         top: FSM, NR loop, shared-multiplier mux, 2 - e through sub32, sqrt, monitor   (Monish / Keerat)
testbench/tb_fp_div.v       self-checking testbench, reads vectors.hex / vectors_sqrt.hex   (Keerat)
testbench/tb_vedic.v        checks vedic 8x8 / 24x24 / 32x32 against '*'
testbench/tb_sub32.v        checks sub32 against '-'
testbench/tb_fault.v        STAGE 4: fault injection campaign (needs -DFAULT_INJ)
model/golden_model.cpp  bit-accurate C++ model + vector generator (divide, sqrt, seed modes, monitor, faults)
plots/plot_stage4.py    STAGE 4: makes the png plots from the csv files
design.sv, testbench.sv combined single files for EDA Playground (same code as the separate files)

## Special cases (Stage 3)
Handled in fp_div.v before the NR loop; special operands skip the datapath and finish in 1 cycle.

| A \ B          | normal      | 0 / subnormal       | Inf          | NaN       |
|----------------|-------------|---------------------|--------------|-----------|
| normal         | NR datapath | +-Inf (div_by_zero) | +-0          | NaN       |
| 0 / subnormal  | +-0         | qNaN (invalid)      | +-0          | NaN       |
| Inf            | +-Inf       | +-Inf               | qNaN (invalid) | NaN     |
| NaN            | NaN         | NaN                 | NaN          | NaN       |

- NaN inputs are propagated (made quiet, A wins over B); a signalling NaN raises invalid. Default NaN = 0x7FC00000.
- Subnormal inputs are treated as zero (DAZ); subnormal results are flushed to zero (FTZ).
- New output `flags[3:0] = {invalid, div_by_zero, overflow, underflow}`, valid with `done`.
- tb_fp_div.v runs 25 directed special-case vectors after the random ones (exact bit + flag match).

## Stage 4
Four additions. With the defaults (`SEED_MODE=0`, `auto_stop=0`, `op=0`) the divider is bit identical to Stage 3.

New ports / parameters of fp_div

| name | dir | meaning |
|------|-----|---------|
| `SEED_MODE` | parameter | 0 = seed_lut (default), 1 = single line 48/17 - 32/17 d, 2 = piecewise linear, 8 segments |
| `op` | in | 0 = a / b, 1 = sqrt(a) (b ignored) |
| `auto_stop` | in | 1 = stop when \|1 - d*x\| < 2^-15, `num_iter` becomes the maximum |
| `iters_used[2:0]` | out | iterations actually done, valid with `done` |
| `converged` | out | 1 = the monitor stopped the loop before `num_iter` |
| `fault_det` | out | 1 = divide failed the final residual check \|1 - d*x\| < 2^-26 (needs enough iterations, e.g. 3 with the LUT) |

1. **Seed modes** (seed_linear.v). x0 = c0 - c1*d with the minimax line per segment, c1*d on vedic_16x16 (top 16 bits of d), c0 - c1*d on sub32, all combinational.
   PWL8 gives 9.15 correct bits (all 2^23 mantissas), so **2 iterations = 8 cycles** with the same accuracy as the LUT at 3 iterations.
2. **Convergence monitor** (conv_monitor.v). e = d*x is already made in S_E; r = e - 1.0 through sub32, |r| < 2^-15 when r[31:15] is all 0s or all 1s.
   The error squares, so that iteration becomes the last one (no extra cycle).
3. **Square root** (op = 1). y(n+1) = y*(3 - x*y*y)/2, sqrt = x*y, 3 multiplies per iteration on the shared multiplier, 16 entry seed.
   The free multiplier in S_OUT checks (2q+1)^2 against 4x, so every result is **correctly rounded**. Latency 4 + 3N = 13 cycles at N = 3.
   sqrt(NaN) = NaN (quietened), sqrt(-x) = 0x7FC00000 + invalid, sqrt(+-0) = +-0, sqrt(+Inf) = +Inf, subnormal -> +-0 (DAZ).
4. **Fault injection** (`ifdef FAULT_INJ`, simulation only). Internal regs fi_en / fi_sel / fi_iter / fi_bit, set from the testbench
   (`dut.fi_en = 1`), flip one bit of x, e, d, mant_A or prod while it is written. The residual check in S_OUT catches 100% of the x / e faults
   that give a wrong result, 0 false alarms; faults in d / mant_A / prod are not caught (NR converges to the wrong 1/d perfectly).

Results (RTL simulation, every RTL run equals the C++ model bit for bit, see reports/Stage4 - change report.md)

| config | vectors | exact | 1 ULP | max ULP | latency (cycles) |
|--------|---------|-------|-------|---------|------------------|
| LUT, 3 it (Stage 3) | 4000 | 3981 | 19 | 1 | 10 |
| single line, 3 it | 4000 | 3981 | 19 | 1 | 10 |
| PWL8, 2 it | 4000 | 3980 | 20 | 1 | 8 |
| LUT, monitor on, cap 3 | 4000 | 3981 | 19 | 1 | 6 / 9.74 / 10 (avg 2.871 it) |
| PWL8, monitor on, cap 3 | 4000 | 3981 | 19 | 1 | 6 / 7.95 / 8 (avg 1.975 it) |
| sqrt, 3 it | 2000 | 2000 | 0 | 0 | 13 (1 for specials) |
| sqrt, monitor on, cap 3 | 2000 | 2000 | 0 | 0 | avg 12.23 |

## Run the C++ model (generates vectors.hex, vectors_sqrt.hex + csv files)
    g++ -O2 -std=c++17 model/golden_model.cpp -o model/golden_model
    ./model/golden_model
  writes vectors.hex (unchanged, 4000 lines), vectors_sqrt.hex (2000 lines), accuracy_vs_iter.csv, convergence.csv,
  monitor.csv, fault_campaign.csv, and prints the exhaustive 2^24 sqrt check. Extra modes:
    ./model/golden_model --dump SEED ITER AUTO OP model.csv     (per-vector hw results, same format as the tb results.csv)
    ./model/golden_model --fault-subset model_fault.csv        (same campaign as tb_fault.v, same format as fault_results.csv)

## Run the Verilog simulation (Icarus Verilog)
    mkdir -p sim && cp vectors.hex vectors_sqrt.hex sim/ && cd sim
    iverilog -g2005 -Wall -o sim.out ../register-level-modules/*.v ../testbench/tb_fp_div.v
    vvp sim.out +ITER=3 +VERBOSE=10
  (Vivado: add register-level-modules/*.v + tb as sources, add vectors.hex as a simulation source,
   or put its full path in $readmemh; plusargs go in Simulation Settings > xsim.simulate.xsim.more_options)

  plusargs: +ITER=n  +VERBOSE=n  +RUN=n (0 = all vectors, default 20)  +AUTO=0/1  +OP=0/1  +SQRT_VEC=file  +CSV=1
  seed mode is a parameter: add -Ptb_fp_div.SEED_MODE=2 to the iverilog line
    vvp sim.out +ITER=3 +RUN=0                      # Stage 3 regression (3981 / 19 / max 1, 25/25)
    iverilog -g2005 -Wall -Ptb_fp_div.SEED_MODE=2 -o pwl.out ../register-level-modules/*.v ../testbench/tb_fp_div.v
    vvp pwl.out +ITER=2 +RUN=0                      # PWL seed, 8 cycles
    vvp sim.out +ITER=3 +AUTO=1 +RUN=0              # convergence monitor
    vvp sim.out +ITER=3 +OP=1 +RUN=0                # square root
    vvp sim.out +ITER=3 +AUTO=1 +RUN=0 +CSV=1 +VERBOSE=6   # results.csv + conv_log.csv for the plots
  EDA Playground: design.sv + testbench.sv + vectors.hex (+ vectors_sqrt.hex), Icarus Verilog, plusargs in "Run Options".

## vedic and sub32 tests (from sim/)
    iverilog -g2005 -Wall -o vedic.out ../register-level-modules/vedic_mul.v ../testbench/tb_vedic.v && vvp vedic.out
    iverilog -g2005 -Wall -o sub32.out ../register-level-modules/sub32.v ../register-level-modules/cla8.v ../testbench/tb_sub32.v && vvp sub32.out

## fault injection campaign (from sim/, ~25 s)
    iverilog -g2005 -Wall -DFAULT_INJ -o fault.out ../register-level-modules/*.v ../testbench/tb_fault.v && vvp fault.out
  writes fault_results.csv (200 vectors, 78 site/iteration/bit points). The full sweep (all bits, 2000 vectors) is fault_campaign.csv from the model.

## plots (from the repo root, after the model and the +CSV=1 run above)
    python plots/plot_stage4.py
  needs python3 + matplotlib + numpy, writes plots/1_correct_bits_vs_iter.png ... plots/6_fault_heatmap.png
