// Copyright (c) Tyler Veness

#include "InitArgs.h"

#include "frc/system/Discretization.h"

void InitArgs(Eigen::Matrix<double, 5, 5>& A, Eigen::Matrix<double, 5, 2>& B,
              Eigen::Matrix<double, 5, 5>& Q, Eigen::Matrix<double, 2, 2>& R) {
  Eigen::Matrix<double, 5, 5> contA{
      {0, 0, 0, 0.5, 0.5},
      {0, 0, 0, 0, 0},
      {0, 0, 0, -1.1111111111111112, 1.1111111111111112},
      {0, 0, 0, -10.486221508345572, 5.782171664108812},
      {0, 0, 0, 5.782171664108812, -10.486221508345572}};
  Eigen::Matrix<double, 5, 2> contB{{0, 0},
                                    {0, 0},
                                    {0, 0},
                                    {6.664631384780125, -5.106998986026231},
                                    {-5.106998986026231, 6.664631384780125}};
  Q = Eigen::Matrix<double, 5, 5>{{256, 0, 0, 0, 0},
                                  {0, 64, 0, 0, 0},
                                  {0, 0, 0.16, 0, 0},
                                  {0, 0, 0, 1.10803324099723, 0},
                                  {0, 0, 0, 0, 1.10803324099723}};
  R = Eigen::Matrix<double, 2, 2>{{0.006944444444444444, 0},
                                  {0, 0.006944444444444444}};

  constexpr double velocity = 2.0;
  contA(1, 2) = velocity;

  frc::DiscretizeAB<5, 2>(contA, contB, 0.005, &A, &B);
}
