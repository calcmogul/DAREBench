// Copyright (c) Tyler Veness

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include "InitArgs.hpp"
#include "wpi/math/linalg/DARE.hpp"

void DARE_WPIMath_Static(benchmark::State& state) {
  Eigen::Matrix<double, 5, 5> A;
  Eigen::Matrix<double, 5, 2> B;
  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 2, 2> R;
  InitArgs(A, B, Q, R);

  for (auto _ : state) {
    auto S = wpi::math::DARE<5, 2>(A, B, Q, R).value();
    benchmark::DoNotOptimize(S);
  }
}
