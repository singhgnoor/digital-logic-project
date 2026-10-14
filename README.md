# fp_div – IEEE-754 single-precision Newton-Raphson divider (Stage 2 core)

rtl/mant_mul.v       shared 32x32 multiplier wrapper ('*' for now, swappable)
rtl/fp_unpack.v      sign / exponent / 24-bit mantissa            (Divya)
rtl/seed_lut.v       8-entry reciprocal seed, Q2.30               (Gurnoor)
rtl/fp_round_pack.v  normalise + round-nearest-even + pack        (Mansi)
rtl/fp_div.v         top: FSM, NR loop, shared-multiplier mux     (Monish / Keerat)
tb/tb_fp_div.v       self-checking testbench, reads vectors.hex   (Keerat)
model/golden_model.cpp  bit-accurate C++ model + vector generator

## Run the C++ model (generates vectors.hex + csv files)
    g++ -O2 -std=c++17 model/golden_model.cpp -o model/golden_model
    ./model/golden_model

## Run the Verilog simulation (Icarus Verilog)
    mkdir -p sim && cp vectors.hex sim/ && cd sim
    iverilog -g2005 -o sim.out ../rtl/*.v ../tb/tb_fp_div.v
    vvp sim.out +ITER=3 +VERBOSE=10
  (Vivado: add rtl/*.v + tb as sources, add vectors.hex as a simulation source,
   or put its full path in $readmemh; plusargs go in Simulation Settings > xsim.simulate.xsim.more_options)

## Not done yet (by design): zero/Inf/NaN/subnormal inputs, sqrt, fault injection,
## convergence monitor, pipelining.
