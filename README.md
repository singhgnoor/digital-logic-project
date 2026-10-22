# fp_div – IEEE-754 single-precision Newton-Raphson divider (Stage 3)

register-level-modules/mant_mul.v       shared 32x32 multiplier wrapper, uses vedic_32x32 (USE_VEDIC=1 default, 0 = plain '*')
register-level-modules/vedic_mul.v      vedic multiplier, 2x2 gate level up to 32x32, plus 24x24 for mantissa
register-level-modules/full_adder.v     1 bit full adder
register-level-modules/cla8.v           8 bit carry lookahead adder
register-level-modules/sub32.v          32 bit subtractor, 4 cla8 blocks with ripple carry between them
register-level-modules/fp_unpack.v      sign / exponent / 24-bit mantissa + zero/sub/inf/nan classification   (Divya)
register-level-modules/seed_lut.v       8-entry reciprocal seed, Q2.30               (Gurnoor)
register-level-modules/fp_round_pack.v  normalise + round-nearest-even + pack        (Mansi)
register-level-modules/fp_div.v         top: FSM, NR loop, shared-multiplier mux, 2 - e through sub32   (Monish / Keerat)
testbench/tb_fp_div.v       self-checking testbench, reads vectors.hex   (Keerat)
testbench/tb_vedic.v        checks vedic 8x8 / 24x24 / 32x32 against '*'
testbench/tb_sub32.v        checks sub32 against '-'
model/golden_model.cpp  bit-accurate C++ model + vector generator

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

## Run the C++ model (generates vectors.hex + csv files)
    g++ -O2 -std=c++17 model/golden_model.cpp -o model/golden_model
    ./model/golden_model

## Run the Verilog simulation (Icarus Verilog)
    mkdir -p sim && cp vectors.hex sim/ && cd sim
    iverilog -g2005 -o sim.out ../register-level-modules/*.v ../testbench/tb_fp_div.v
    vvp sim.out +ITER=3 +VERBOSE=10
  (Vivado: add register-level-modules/*.v + tb as sources, add vectors.hex as a simulation source,
   or put its full path in $readmemh; plusargs go in Simulation Settings > xsim.simulate.xsim.more_options)

## vedic and sub32 tests (from sim/)
    iverilog -g2005 -o vedic.out ../register-level-modules/vedic_mul.v ../testbench/tb_vedic.v && vvp vedic.out
    iverilog -g2005 -o sub32.out ../register-level-modules/sub32.v ../register-level-modules/cla8.v ../testbench/tb_sub32.v && vvp sub32.out
