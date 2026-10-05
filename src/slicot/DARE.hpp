// Copyright (c) Tyler Veness

#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

extern "C" {

// https://gcc.gnu.org/onlinedocs/gfortran/Argument-passing-conventions.html
void sb02od_(
    // Mode parameters
    char* DICO, char* JOBB, char* FACT, char* UPLO, char* JOBL, char* SORT,
    // Input/output parameters
    int* N, int* M, int* P, double* A, int* LDA, double* B, int* LDB, double* Q,
    int* LDQ, double* R, int* LDR, double* L, int* LDL, double* RCOND,
    double* X, int* LDX, double* ALFAR, double* ALFAI, double* BETA, double* S,
    int* LDS, double* T, int* LDT, double* U, int* LDU,
    // Tolerances
    double* TOL,
    // Workspace
    int* IWORK, double* DWORK, int* LDWORK, int* BWORK,
    // Error indicator
    int* INFO,
    // Hidden arguments for charlen
    size_t, size_t, size_t, size_t, size_t, size_t);

}  // extern "C"

namespace slicot {

/**
 * Computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 *   AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @throws std::invalid_argument if SLICOT fails.
 */
template <int States, int Inputs>
Eigen::Matrix<double, States, States> DARE(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  // Discrete-time
  char DICO = 'D';

  // B and R are given
  char JOBB = 'B';

  // Not factored, Q and R are given
  char FACT = 'N';

  // Upper triangle of Q and R are stored
  char UPLO = 'U';

  // L is zero
  char JOBL = 'Z';

  // Stable eigenvalues come first
  char SORT = 'S';

  // (input) State dimension
  int N = States;

  // (input) Number of system inputs
  int M = Inputs;

  // (input) Number of system outputs (not used because FACT is 'N')
  int P = 0;

  // (input) State matrix of the system A
  Eigen::Matrix<double, States, States> A_copy = A;

  // (input) Input matrix of the system B
  Eigen::Matrix<double, States, Inputs> B_copy = B;

  // (input) Symmetric state weighting matrix Q
  Eigen::Matrix<double, States, States> Q_copy = Q;

  // (input) Symmetric input weighting matrix R
  Eigen::Matrix<double, Inputs, Inputs> R_copy = R;

  // Leading dimension of array L (cross weighting matrix)
  int LDL = 1;

  // (output) Estimate of reciprocal of condition number
  double RCOND = 0.0;

  // (output) Solution matrix X of the problem
  Eigen::Matrix<double, States, States> X;

  // (output)
  double ALFAR[2 * States];

  // (output)
  double ALFAI[2 * States];

  // (output) Generalized eigenvalues of the (2×States)×(2×States) matrix pair
  double BETA[2 * States];

  // (output) Schur form of S
  Eigen::Matrix<double, 2 * States + Inputs, 2 * States + Inputs> S;

  // Leading dimension of array S
  int LDS = 2 * States + Inputs;

  // (output) Ordered upper triangular form T of the second matrix in the
  // reduced matrix pencil associated to the optimal problem
  Eigen::Matrix<double, 2 * States + Inputs, 2 * States> T;

  // Leading dimension of array T
  int LDT = 2 * States + Inputs;

  // (output) Right transformation U which reduces the 2N-by-2N matrix pencil to
  // the ordered generalized real Schur form (S, T)
  Eigen::Matrix<double, 2 * States, 2 * States> U;

  // Leading dimension of array U
  int LDU = 2 * States;

  // Tolerance to be used to test for near singularity of original matrix pencil
  double TOL = 0.0;

  // Workspace
  int IWORK[std::max(2 * States, Inputs)];
  double DWORK[16 * States + 3 * Inputs + 16];
  int LDWORK = 16 * States + 3 * Inputs + 16;
  int BWORK[2 * States];

  // = 0:  successful exit;
  // < 0:  if INFO = -i, the i-th argument had an illegal
  //       value;
  // = 1:  if the computed extended matrix pencil is singular,
  //       possibly due to rounding errors;
  // = 2:  if the QZ (or QR) algorithm failed;
  // = 3:  if reordering of the (generalized) eigenvalues
  //       failed;
  // = 4:  if after reordering, roundoff changed values of
  //       some complex eigenvalues so that leading eigenvalues
  //       in the (generalized) Schur form no longer satisfy
  //       the stability condition; this could also be caused
  //       due to scaling;
  // = 5:  if the computed dimension of the solution does not
  //       equal N;
  // = 6:  if a singular matrix was encountered during the
  //       computation of the solution matrix X.
  int INFO;

  sb02od_(&DICO, &JOBB, &FACT, &UPLO, &JOBL, &SORT, &N, &M, &P, A_copy.data(),
          &N, B_copy.data(), &N, Q_copy.data(), &N, R_copy.data(), &M, nullptr,
          &LDL, &RCOND, X.data(), &N, ALFAR, ALFAI, BETA, S.data(), &LDS,
          T.data(), &LDT, U.data(), &LDU, &TOL, IWORK, DWORK, &LDWORK, BWORK,
          &INFO, 1, 1, 1, 1, 1, 1);

  if (INFO < 0) {
    throw std::runtime_error("DARE failed: INFO = " + std::to_string(INFO));
  }

  return X;
}

}  // namespace slicot
