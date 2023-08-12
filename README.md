# DARE benchmark

This repository runs benchmarks of various DARE solvers.

## Build instructions

### Dependencies

Install the following packages: cmake, gcc, gfortran, blas, lapack.

### Build and run

```bash
cmake -B build -S .
cmake --build build
./build/DAREBench
```

## Results

```
Running ./build/DAREBench
Run on (8 X 3600.24 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 6144 KiB (x1)
Load Average: 0.92, 1.00, 1.00
-------------------------------------------------------
Benchmark             Time             CPU   Iterations
-------------------------------------------------------
DARE_WPIMath      28064 ns        28045 ns        25089
DARE_SLICOT       41198 ns        41169 ns        16995
DARE_Drake        91757 ns        91678 ns         7658
```

## Licensing

|Files           |Upstream Repo                             |License            |
|----------------|------------------------------------------|-------------------|
|`src/frc/`      |https://github.com/wpilibsuite/allwpilib  |WPILib 3-Clause BSD|
|`src/slicot/*.f`|https://github.com/SLICOT/SLICOT-Reference|SLICOT 3-Clause BSD|
|`src/drake/`    |https://github.com/RobotLocomotion/drake  |Drake 3-Clause BSD |
|everything else |N/A                                       |3-Clause BSD       |
