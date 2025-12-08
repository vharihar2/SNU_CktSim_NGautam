#include <gtest/gtest.h>

#include "NonlinearInductor.hpp"
#include "NonlinearModel.hpp"

#include "../lib/external/Eigen/Dense"

#include <cmath>
#include <map>
#include <vector>
#include <string>

/*
 * nonlinear_inductor_test.cpp
 *
 * Unit tests for NonlinearInductor and polynomial flux models.
 *
 * Tests cover:
 *  - Polynomial flux model evaluation (phi) and derivative (dphidi)
 *  - Per-Newton companion linearization via computeCompanionIter(...)
 *    and subsequent stamping via stampTransient(...)
 *  - Branch-index stamping (node <-> branch coupling and branch diagonal)
 *  - Fallback node-based stamping when branch current unknown absent
 *  - Update state from solution reading branch-current when present
 *  - Snapshot/restore behavior
 *
 * Tests assert observable behavior (MNA matrix and RHS), not internal private
 * members.
 */

// Helper to create a zeroed square matrix for MNA of size n
static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
  return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

TEST(PolynomialFluxModel, EvalPhiAndDerivative)
{
  // phi(i) = b0 + b1*i + b2*i^2
  std::vector<double> coeffs = {0.0, 1e-3, 1e-4}; // b1 = 1e-3 H, small nonlinear term
  auto model = makePolynomialFluxModel(coeffs);

  double i = 0.2; // amps
  double phi = model->phi(i);
  double dphi = model->dphidi(i);

  // phi = b1*i + b2*i^2 = 1e-3*0.2 + 1e-4*(0.04) = 2e-4 + 4e-6 = 2.04e-4
  EXPECT_NEAR(phi, 2.04e-4, 1e-12);

  // dphidi = b1 + 2*b2*i = 1e-3 + 2*1e-4*0.2 = 1e-3 + 4e-5 = 0.00104
  EXPECT_NEAR(dphi, 0.00104, 1e-12);
}

TEST(NonlinearInductor, BranchCompanionIterAndStampTransient)
{
  // Linear flux polynomial phi(i) = L * i (coeffs b0=0, b1=L)
  double L = 10e-3; // 10 mH
  auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
  auto ind = std::make_shared<NonlinearInductor>("L_lin", "P", "N", L, model);

  // Index map contains node indices and branch-current unknown under the element name
  std::map<std::string,int> indexMap;
  indexMap["P"] = 0;
  indexMap["N"] = 1;
  indexMap["L_lin"] = 2; // branch current index

  // Newton iterate/solution guess: node voltages + branch current
  Eigen::VectorXd xk(3);
  xk[0] = 0.5;   // vP
  xk[1] = -0.2;  // vN
  xk[2] = 0.05;  // i_k

  double h = 1e-3;

  // compute per-iterate companion linearization
  ind->computeCompanionIter(h, xk, indexMap);

  // Prepare MNA and RHS
  auto mna = makeZeroMNA(3);
  std::vector<double> rhs(3, 0.0);

  ind->stampTransient(mna, rhs, indexMap);

  // For linear phi = L*i: dphidi = L => Geq = h / (2 * L)
  double expectedGeq = h / (2.0 * L);
  double expectedRseries = 1.0 / expectedGeq;

  // Check node <-> branch couplings exist
  EXPECT_DOUBLE_EQ(mna[0][2], 1.0);
  EXPECT_DOUBLE_EQ(mna[1][2], -1.0);
  EXPECT_DOUBLE_EQ(mna[2][0], 1.0);
  EXPECT_DOUBLE_EQ(mna[2][1], -1.0);

  // Branch diagonal should include -Rseries (per implementation)
  EXPECT_DOUBLE_EQ(mna[2][2], -expectedRseries);

  // RHS branch entry should be finite
  EXPECT_TRUE(std::isfinite(rhs[2]));
}

TEST(NonlinearInductor, FallbackNodeBasedStampingWhenBranchMissing)
{
  double L = 5e-3; // 5 mH
  auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
  auto ind = std::make_shared<NonlinearInductor>("L_nb", "A", "B", L, model);

  // indexMap WITHOUT branch index; only node indices
  std::map<std::string,int> indexMap;
  indexMap["A"] = 0;
  indexMap["B"] = 1;

  // iterate vector containing only node voltages
  Eigen::VectorXd xk(2);
  xk[0] = 1.0;
  xk[1] = 0.25;

  double h = 2e-3;
  ind->computeCompanionIter(h, xk, indexMap);

  auto mna = makeZeroMNA(2);
  std::vector<double> rhs(2, 0.0);
  ind->stampTransient(mna, rhs, indexMap);

  // Node-based Norton stamping should have produced symmetric conductance entries
  EXPECT_NE(mna[0][0], 0.0);
  EXPECT_NE(mna[0][1], 0.0);
  EXPECT_NE(mna[1][0], 0.0);
  EXPECT_NE(mna[1][1], 0.0);

  // RHS values should be finite and opposite in sign (rhs[na] = -rhs[nb])
  EXPECT_TRUE(std::isfinite(rhs[0]));
  EXPECT_TRUE(std::isfinite(rhs[1]));
  EXPECT_DOUBLE_EQ(rhs[0], -rhs[1]);
}

TEST(NonlinearInductor, UpdateStateReadsBranchCurrentWhenPresentAndSnapshotRestore)
{
  double L = 2e-3;
  auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
  auto ind = std::make_shared<NonlinearInductor>("L_up", "P", "M", L, model);

  std::map<std::string,int> indexMap;
  indexMap["P"] = 0;
  indexMap["M"] = 1;
  indexMap["L_up"] = 2;

  // Solution vector with branch current known
  Eigen::VectorXd x(3);
  x[0] = 0.8;   // vP
  x[1] = 0.3;   // vM
  x[2] = 0.123; // branch current i_{n+1}

  // Update internal state from solution (should read branch current)
  ind->updateStateFromSolution(x, indexMap);

  // Snapshot the state and restore into a fresh element instance
  auto snap = ind->snapshotState();
  auto ind2 = std::make_shared<NonlinearInductor>("L_rest", "P", "M", L, model);
  ind2->restoreState(snap);

  // Recompute companion and stamp to verify no crash and finite outputs
  ind2->computeCompanion(1e-3);
  auto mna = makeZeroMNA(3);
  std::vector<double> rhs(3, 0.0);
  ind2->stampTransient(mna, rhs, indexMap);

  EXPECT_TRUE(std::isfinite(rhs[2]));
  EXPECT_TRUE(std::isfinite(mna[2][2]));
}

TEST(NonlinearInductor, ComputeCompanionHandlesZeroTimestepGracefully)
{
  double L = 1e-3;
  auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
  auto ind = std::make_shared<NonlinearInductor>("L_zero", "X", "Y", L, model);

  std::map<std::string,int> indexMap;
  indexMap["X"] = 0;
  indexMap["Y"] = 1;
  indexMap["L_zero"] = 2;

  // zero timestep should set Geq_ and Ieq_ to zero and stampTransient should
  // not modify MNA/RHS
  ind->computeCompanion(0.0);

  auto mna = makeZeroMNA(3);
  std::vector<double> rhs(3, 0.0);
  ind->stampTransient(mna, rhs, indexMap);

  for (size_t i = 0; i < mna.size(); ++i) {
    for (size_t j = 0; j < mna[i].size(); ++j) {
      EXPECT_DOUBLE_EQ(mna[i][j], 0.0);
    }
    EXPECT_DOUBLE_EQ(rhs[i], 0.0);
  }
}

// End of file - no main() here; gtest_main provides test runner.