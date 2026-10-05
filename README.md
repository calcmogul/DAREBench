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

### Linux x86-64 desktop (AMD Ryzen 7840U)

Ran via `./build/DAREBench --benchmark_time_unit=us`.

```
Running ./build/DAREBench
Run on (16 X 3253.98 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x8)
  L1 Instruction 32 KiB (x8)
  L2 Unified 1024 KiB (x8)
  L3 Unified 16384 KiB (x1)
Load Average: 0.42, 0.52, 0.51
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
DARE_WPIMath_Dynamic                       16.2 us         16.2 us        44050
DARE_WPIMath_NoPrecondChecks_Dynamic       11.6 us         11.6 us        57903
DARE_WPIMath_Static                        8.49 us         8.48 us        79163
DARE_WPIMath_NoPrecondChecks_Static        5.80 us         5.80 us       119024
DARE_SLICOT                                21.7 us         21.6 us        31424
DARE_Drake                                 16.3 us         16.3 us        42643
```

### Raspberry Pi 5

Ran via `./build/DAREBench --benchmark_time_unit=us`.

```
Running ./build/DAREBench
Run on (4 X 2400 MHz CPU s)
CPU Caches:
  L1 Data 64 KiB (x4)
  L1 Instruction 64 KiB (x4)
  L2 Unified 512 KiB (x4)
  L3 Unified 2048 KiB (x1)
Load Average: 0.47, 0.72, 0.45
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
-------------------------------------------------------------------------------
Benchmark                                     Time             CPU   Iterations
-------------------------------------------------------------------------------
DARE_WPIMath_Dynamic                       34.4 us         34.4 us        20315
DARE_WPIMath_NoPrecondChecks_Dynamic       21.7 us         21.7 us        32266
DARE_WPIMath_Static                        15.2 us         15.2 us        45878
DARE_WPIMath_NoPrecondChecks_Static        7.84 us         7.84 us        89316
DARE_SLICOT                                79.4 us         79.4 us         8789
DARE_Drake                                 34.9 us         34.9 us        20074
```

## Licensing

|Files           |Upstream Repo                             |License            |
|----------------|------------------------------------------|-------------------|
|`src/frc/`      |https://github.com/wpilibsuite/allwpilib  |WPILib 3-Clause BSD|
|`src/slicot/*.f`|https://github.com/SLICOT/SLICOT-Reference|SLICOT 3-Clause BSD|
|`src/drake/`    |https://github.com/RobotLocomotion/drake  |Drake 3-Clause BSD |
|everything else |N/A                                       |3-Clause BSD       |
