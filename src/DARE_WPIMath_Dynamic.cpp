// Copyright (c) Tyler Veness

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include "InitArgs.h"
#include "frc/DARE.h"

void DARE_WPIMath_Dynamic(benchmark::State& state) {
  Eigen::Matrix<double, 5, 5> A;
  Eigen::Matrix<double, 5, 2> B;
  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 2, 2> R;
  InitArgs(A, B, Q, R);

  for (auto _ : state) {
    frc::DARE<Eigen::Dynamic, Eigen::Dynamic>(A, B, Q, R);
  }
}
