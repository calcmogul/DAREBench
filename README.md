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
2023-08-12T12:06:24-07:00
Running ./build/DAREBench
Run on (8 X 3599.72 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 6144 KiB (x1)
Load Average: 3.20, 2.53, 1.74
----------------------------------------------------------------
Benchmark                      Time             CPU   Iterations
----------------------------------------------------------------
DARE_WPIMath               27410 ns        27389 ns        25397
DARE_WPIMath_Internal      20542 ns        20528 ns        33928
DARE_SLICOT                41685 ns        41651 ns        16829
DARE_Drake                 92195 ns        92098 ns         7582
```

## Licensing

|Files           |Upstream Repo                             |License            |
|----------------|------------------------------------------|-------------------|
|`src/frc/`      |https://github.com/wpilibsuite/allwpilib  |WPILib 3-Clause BSD|
|`src/slicot/*.f`|https://github.com/SLICOT/SLICOT-Reference|SLICOT 3-Clause BSD|
|`src/drake/`    |https://github.com/RobotLocomotion/drake  |Drake 3-Clause BSD |
|everything else |N/A                                       |3-Clause BSD       |
