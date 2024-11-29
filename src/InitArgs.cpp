// Copyright (c) Tyler Veness

#include "InitArgs.h"

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template <int States, int Inputs>
void DiscretizeAB(const Eigen::Matrix<double, States, States>& contA,
                  const Eigen::Matrix<double, States, Inputs>& contB, double dt,
                  Eigen::Matrix<double, States, States>* discA,
                  Eigen::Matrix<double, States, Inputs>* discB) {
  // M = [A  B]
  //     [0  0]
  Eigen::Matrix<double, States + Inputs, States + Inputs> M;
  M.template block<States, States>(0, 0) = contA;
  M.template block<States, Inputs>(0, States) = contB;
  M.template block<Inputs, States + Inputs>(States, 0).setZero();

  // ϕ = eᴹᵀ = [A_d  B_d]
  //           [ 0    I ]
  Eigen::Matrix<double, States + Inputs, States + Inputs> phi = (M * dt).exp();

  *discA = phi.template block<States, States>(0, 0);
  *discB = phi.template block<States, Inputs>(0, States);
}

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

  DiscretizeAB<5, 2>(contA, contB, 0.005, &A, &B);
}
