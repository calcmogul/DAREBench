// Copyright (c) Tyler Veness

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include "InitArgs.hpp"
#include "frc/DARE.hpp"

void DARE_WPIMath_NoPrecondChecks_Dynamic(benchmark::State& state) {
  Eigen::Matrix<double, 5, 5> A;
  Eigen::Matrix<double, 5, 2> B;
  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 2, 2> R;
  InitArgs(A, B, Q, R);

  for (auto _ : state) {
    frc::DARE<Eigen::Dynamic, Eigen::Dynamic>(A, B, Q, R, false);
  }
}
