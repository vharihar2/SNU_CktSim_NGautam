/*
 * SNU_Spice/tests/elements/nonlinear/nonlinear_capacitor_test.cpp
 *
 * Unit tests focused on NonlinearCapacitor and its polynomial charge models.
 *
 * The test cases verify:
 *  - Polynomial model evaluation and derivative correctness for small polynomials.
 *  - Per-Newton companion linearization via computeCompanionIter(...) and
 *    subsequent stamping into MNA/RHS via stampTransient(...).
 *  - Derivative clamping behavior when the model derivative is zero/small.
 *  - Rejection behavior when the computed Geq would be excessively large.
 *  - Snapshot / restore and updateStateFromSolution paths exercise without
 *    accessing private members (effects validated via public stamping).
 *
 * These tests avoid reading internal (private) members and instead assert
 * observable effects on MNA/RHS matrices produced by the public API.
 */

#include <gtest/gtest.h>

#include "NonlinearCapacitor.hpp"
#include "NonlinearModel.hpp"

#include "../lib/external/Eigen/Dense"

#include <cmath>
#include <map>
#include <vector>

// Helper to create a zeroed square MNA matrix of size n
static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
  return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

// ---------- Polynomial model sanity checks ----------
TEST(PolynomialChargeModel, EvaluationAndDerivativeSmallPolynomial)
{
  // q(u) = a0 + a1*u + a2*u^2
  std::vector<double> coeffs = {0.0, 1e-6, 1e-9}; // a0=0, a1=1u, a2=1n
  auto model = makePolynomialChargeModel(coeffs);

  double u = 2.0; // volts
  double q = model->q(u);
  double dqdu = model->dqdu(u);

  // q = a1*u + a2*u^2 = 1e-6*2 + 1e-9*4 = 2e-6 + 4e-9
  EXPECT_NEAR(q, 2e-6 + 4e-9, 1e-12);

  // dqdu = a1 + 2*a2*u = 1e-6 + 2*1e-9*2 = 1e-6 + 4e-9
  EXPECT_NEAR(dqdu, 1e-6 + 4e-9, 1e-15);
}

// ---------- NonlinearCapacitor companion & stamping ----------
TEST(NonlinearCapacitorIter, ComputeCompanionIterAndStampTransient)
{
  // Build a polynomial charge model with a slight quadratic nonlinearity
  std::vector<double> coeffs = {0.0, 1e-6, 1e-8}; // q = 1e-6*u + 1e-8*u^2
  auto model = makePolynomialChargeModel(coeffs);

  // Create element Cnl between nodes A and B with linear term value=1e-6
  auto el = std::make_shared<NonlinearCapacitor>("CNL", "A", "B", 1e-6, model);

  // Prepare an indexMap for nodes A,B
  std::map<std::string,int> indexMap;
  indexMap["A"] = 0;
  indexMap["B"] = 1;

  // Newton iterate vector xk with vA=1.0, vB=0.2 -> u_k = 0.8 V
  Eigen::VectorXd xk(2);
  xk[0] = 1.0;
  xk[1] = 0.2;

  double h = 1e-3;

  // computeCompanionIter should compute Geq_ and Ieq_ (internally) without crashing
  el->computeCompanionIter(h, xk, indexMap);

  // After computeCompanionIter, stamping into an MNA should reflect Geq/Ieq
  auto mna = makeZeroMNA(2);
  std::vector<double> rhs(2, 0.0);
  el->stampTransient(mna, rhs, indexMap);

  // Expect non-zero conductance entries (since dqdu > 0 for the polynomial)
  EXPECT_NE(mna[0][0], 0.0);
  EXPECT_NE(mna[1][1], 0.0);
  EXPECT_NE(mna[0][1], 0.0);
  EXPECT_NE(mna[1][0], 0.0);

  // RHS entries should be finite numbers
  EXPECT_TRUE(std::isfinite(rhs[0]));
  EXPECT_TRUE(std::isfinite(rhs[1]));
}

TEST(NonlinearCapacitorState, SnapshotRestoreAndUpdateStateFromSolution)
{
  std::vector<double> coeffs = {0.0, 1e-6};
  auto model = makePolynomialChargeModel(coeffs);
  auto el = std::make_shared<NonlinearCapacitor>("Csave", "1", "0", 1e-6, model);

  // simulate a state: update from solution where node '1' = 3.0 V
  std::map<std::string,int> idx;
  idx["1"] = 0;
  Eigen::VectorXd x(1);
  x[0] = 3.0;

  // computeCompanion to initialize any internal helpers and then update state
  el->computeCompanion(1e-3);
  el->updateStateFromSolution(x, idx);

  // snapshot then restore into a new element and ensure restore doesn't crash
  auto s = el->snapshotState();
  auto el2 = std::make_shared<NonlinearCapacitor>("Crestore", "1", "0", 1e-6, model);
  el2->restoreState(s);

  // After restore, recompute companion and stampTransient to ensure no crash
  std::vector<std::vector<double>> mna = makeZeroMNA(1);
  std::vector<double> rhs(1, 0.0);
  el2->computeCompanion(1e-3);
  el2->stampTransient(mna, rhs, idx);

  SUCCEED(); // Reaching here without crash is considered success for state restore path
}

// ---------- Safety and edge-case behaviors ----------

TEST(NonlinearCapacitor, DerivativeClampingAtMinDeriv)
{
  // Polynomial q(u) = a0 (constant), so dqdu = 0 -> should be clamped to minDeriv
  auto model = makePolynomialChargeModel(std::vector<double>{1.0}); // a0 = 1.0 (constant)
  auto cap = std::make_shared<NonlinearCapacitor>("C_clamp", "P", "N", 1.0, model);

  std::map<std::string,int> indexMap;
  indexMap["P"] = 0;
  indexMap["N"] = 1;

  Eigen::VectorXd xk(2);
  xk[0] = 0.2;
  xk[1] = 0.1; // u_k = 0.1

  double h = 1e-3;

  cap->computeCompanionIter(h, xk, indexMap);

  auto mna = makeZeroMNA(2);
  std::vector<double> rhs(2, 0.0);

  cap->stampTransient(mna, rhs, indexMap);

  // The implementation clamps minDeriv = 1e-12
  const double minDeriv = 1e-12;
  double expectedGeq = 2.0 * minDeriv / h;

  // Allow a small relative tolerance for rounding
  EXPECT_NEAR(mna[0][0], expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
  EXPECT_NEAR(mna[1][1], expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
  EXPECT_NEAR(mna[0][1], -expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
  EXPECT_NEAR(mna[1][0], -expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
}

TEST(NonlinearCapacitor, RejectsTooLargeGeqAndAvoidsStamping)
{
  // Create a model with a very large derivative (dqdu) so that Geq_temp > maxGeq
  // in computeCompanionIter and the method will return without accepting new Geq_/Ieq_.
  double largeA1 = 1e9; // derivative will be ~1e9
  auto model = makePolynomialChargeModel(std::vector<double>{0.0, largeA1});
  auto cap = std::make_shared<NonlinearCapacitor>("C_bad", "P", "N", 1.0, model);

  std::map<std::string,int> indexMap;
  indexMap["P"] = 0;
  indexMap["N"] = 1;

  Eigen::VectorXd xk(2);
  xk[0] = 1.0;
  xk[1] = 0.0; // u_k = 1.0

  double h = 1e-3;

  // computeCompanionIter should detect Geq_temp > maxGeq and therefore not accept it.
  cap->computeCompanionIter(h, xk, indexMap);

  auto mna = makeZeroMNA(2);
  std::vector<double> rhs(2, 0.0);
  cap->stampTransient(mna, rhs, indexMap);

  // No stamping should have occurred because Geq_ should remain zero after rejection
  EXPECT_DOUBLE_EQ(mna[0][0], 0.0);
  EXPECT_DOUBLE_EQ(mna[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mna[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mna[1][1], 0.0);
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

// End of file - no main(): gtest_main supplies the test runner.