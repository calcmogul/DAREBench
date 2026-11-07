// Copyright (c) Tyler Veness

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <contrib/Eigen/Riccati>

#include "InitArgs.hpp"

void DARE_Eigen_Static(benchmark::State& state) {
  Eigen::Matrix<double, 5, 5> A;
  Eigen::Matrix<double, 5, 2> B;
  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 2, 2> R;
  InitArgs(A, B, Q, R);

  for (auto _ : state) {
    Eigen::DiscreteAlgebraicRiccatiSolver<Eigen::Matrix<double, 5, 5>> solver{
        A, B, Q, R};
    auto S = solver.solution();
    benchmark::DoNotOptimize(S);
  }
}
