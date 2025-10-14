// Copyright (c) Tyler Veness

#include <Eigen/Core>
#include <benchmark/benchmark.h>

#include "InitArgs.hpp"
#include "frc2/DARE.hpp"

void DARE_WPIMath2_Dynamic(benchmark::State& state) {
  Eigen::Matrix<double, 5, 5> A;
  Eigen::Matrix<double, 5, 2> B;
  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 2, 2> R;
  InitArgs(A, B, Q, R);

  for (auto _ : state) {
    frc2::DARE<Eigen::Dynamic, Eigen::Dynamic>(A, B, Q, R);
  }
}
