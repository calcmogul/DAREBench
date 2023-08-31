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

### x86-64 desktop

Ran via `./build/DAREBench --benchmark_time_unit=us`.

```
Running ./build/DAREBench
Run on (8 X 3599.99 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 6144 KiB (x1)
Load Average: 4.34, 2.23, 1.39
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
DARE_WPIMath_Dynamic                       28.1 us         28.1 us        25090
DARE_WPIMath_NoPrecondChecks_Dynamic       21.1 us         21.0 us        33278
DARE_WPIMath_Static                        13.1 us         13.1 us        53207
DARE_WPIMath_NoPrecondChecks_Static        9.02 us         9.01 us        77794
DARE_SLICOT                                41.8 us         41.8 us        16256
DARE_Drake                                 28.5 us         28.4 us        24502
```

### roboRIO

Ran via `LD_LIBRARY_PATH=. ./DAREBench --benchmark_time_unit=us`.

```
Running ./DAREBench
Run on (2 X 666.66 MHz CPU s)
Load Average: 0.43, 0.29, 0.16
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
DARE_WPIMath_Dynamic                        641 us          640 us         1092
DARE_WPIMath_NoPrecondChecks_Dynamic        433 us          432 us         1618
DARE_WPIMath_Static                         289 us          289 us         2426
DARE_WPIMath_NoPrecondChecks_Static         188 us          188 us         3723
DARE_SLICOT                                 827 us          826 us          848
DARE_Drake                                 2331 us         2328 us          301
```

## Licensing

|Files           |Upstream Repo                             |License            |
|----------------|------------------------------------------|-------------------|
|`src/frc/`      |https://github.com/wpilibsuite/allwpilib  |WPILib 3-Clause BSD|
|`src/slicot/*.f`|https://github.com/SLICOT/SLICOT-Reference|SLICOT 3-Clause BSD|
|`src/drake/`    |https://github.com/RobotLocomotion/drake  |Drake 3-Clause BSD |
|everything else |N/A                                       |3-Clause BSD       |
