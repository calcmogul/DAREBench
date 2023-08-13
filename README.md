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
2023-08-13T12:11:03-07:00
Running ./build/DAREBench
Run on (8 X 3600.02 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 6144 KiB (x1)
Load Average: 1.35, 0.94, 1.31
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
DARE_WPIMath_Dynamic                      26885 ns        26867 ns        26254
DARE_WPIMath_NoPrecondChecks_Dynamic      19504 ns        19492 ns        36113
DARE_WPIMath_Static                       15733 ns        15724 ns        44393
DARE_WPIMath_NoPrecondChecks_Static       10972 ns        10965 ns        62647
DARE_SLICOT                               41877 ns        41819 ns        16882
DARE_Drake                                99919 ns        99834 ns         7021
```

## Licensing

|Files           |Upstream Repo                             |License            |
|----------------|------------------------------------------|-------------------|
|`src/frc/`      |https://github.com/wpilibsuite/allwpilib  |WPILib 3-Clause BSD|
|`src/slicot/*.f`|https://github.com/SLICOT/SLICOT-Reference|SLICOT 3-Clause BSD|
|`src/drake/`    |https://github.com/RobotLocomotion/drake  |Drake 3-Clause BSD |
|everything else |N/A                                       |3-Clause BSD       |
