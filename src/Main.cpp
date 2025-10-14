// Copyright (c) Tyler Veness

#include <benchmark/benchmark.h>

void DARE_WPIMath_Dynamic(benchmark::State& state);
BENCHMARK(DARE_WPIMath_Dynamic);

void DARE_WPIMath_NoPrecondChecks_Dynamic(benchmark::State& state);
BENCHMARK(DARE_WPIMath_NoPrecondChecks_Dynamic);

void DARE_WPIMath_Static(benchmark::State& state);
BENCHMARK(DARE_WPIMath_Static);

void DARE_WPIMath_NoPrecondChecks_Static(benchmark::State& state);
BENCHMARK(DARE_WPIMath_NoPrecondChecks_Static);

void DARE_WPIMath2_Dynamic(benchmark::State& state);
BENCHMARK(DARE_WPIMath2_Dynamic);

void DARE_WPIMath2_NoPrecondChecks_Dynamic(benchmark::State& state);
BENCHMARK(DARE_WPIMath2_NoPrecondChecks_Dynamic);

void DARE_WPIMath2_Static(benchmark::State& state);
BENCHMARK(DARE_WPIMath2_Static);

void DARE_WPIMath2_NoPrecondChecks_Static(benchmark::State& state);
BENCHMARK(DARE_WPIMath2_NoPrecondChecks_Static);

#ifndef _WIN32
void DARE_SLICOT(benchmark::State& state);
BENCHMARK(DARE_SLICOT);
#endif

void DARE_Drake(benchmark::State& state);
BENCHMARK(DARE_Drake);

BENCHMARK_MAIN();
