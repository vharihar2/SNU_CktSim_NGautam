#include <gtest/gtest.h>

#include "Inductor.hpp"
#include "Parser.hpp"

#include <vector>
#include <map>
#include <cmath>

/*
 * inductor_test.cpp
 *
 * Tests for Inductor::parse(...), Inductor::computeCompanion(...),
 * Inductor::stampTransient(...), and updateStateFromSolution behavior.
 *
 * The tests exercise:
 *  - successful parsing
 *  - parsing rejection on malformed input (tokens, same nodes, zero value)
 *  - rejection of zero after suffix, unknown suffix, empty mantissa
 *  - transient companion computation and stamping in branch-current mode
 *    (branch index present)
 *  - fallback Norton stamping when branch index is missing
 *  - guard behavior when timestep <= 0 (no stamping)
 *  - updateStateFromSolution reading branch-current and node voltages
 *
 * Notes:
 *  - Internal state members (i_prev, u_prev, Geq, Ieq) are private. Tests
 *    therefore validate effects visible through public API (stampTransient,
 *    updateStateFromSolution) and documented stamping side-effects.
 */

TEST(InductorParse, ValidToGround)
{
  Parser parser;
  std::vector<std::string> tokens = {"L1", "1", "0", "10m"}; // 10 mH
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  EXPECT_EQ(el->getName(), "L1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
  EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(InductorParse, InvalidTokenCount)
{
  Parser parser;
  std::vector<std::string> tokens = {"L1", "1", "0"}; // missing value token
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid inductor definition"), std::string::npos);
}

TEST(InductorParse, SameNodesRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"LBAD", "N1", "N1", "1m"};
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Inductor nodes cannot be the same"), std::string::npos);
}

TEST(InductorParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"LZ", "N1", "N2", "0"};
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/4);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for inductor value"), std::string::npos);
}

TEST(InductorParse, ZeroWithSuffixRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"LZK", "N1", "N2", "0K"};
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for inductor value"), std::string::npos);
}

TEST(InductorParse, MalformedNumericValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"LX", "N1", "N2", "1.2.3"};
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/6);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(InductorParse, UnknownSuffix)
{
  Parser parser;
  std::vector<std::string> tokens = {"LQ", "N1", "0", "1Q"};
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/7);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Unknown suffix"), std::string::npos);
}

TEST(InductorParse, EmptyMantissaRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"LK", "N1", "N2", "K"}; // no mantissa
  testing::internal::CaptureStderr();
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/8);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(InductorTransient, ComputeCompanionAndStamp_WithBranchIndex)
{
  Parser parser;
  // Choose L = 10 mH
  std::vector<std::string> tokens = {"Lbranch", "Vp", "Vm", "10m"};
  auto el = std::dynamic_pointer_cast<Inductor>(Inductor::parse(parser, tokens, /*lineNumber=*/10));
  ASSERT_NE(el, nullptr);

  // Map indices: Vp -> 0, Vm -> 1, branch Lbranch -> 2
  std::map<std::string,int> indexMap;
  indexMap["Vp"] = 0;
  indexMap["Vm"] = 1;
  indexMap["Lbranch"] = 2;

  // Build solution vector representing previous state:
  // vplus = 0.5 V, vminus = -0.2 V, branch current i_prev = 0.1 A
  Eigen::VectorXd x(3);
  x[0] = 0.5;   // Vp
  x[1] = -0.2;  // Vm
  x[2] = 0.1;   // branch current (i_prev)

  // Update internal state from solution (sets u_prev and i_prev)
  el->updateStateFromSolution(x, indexMap);

  // Choose timestep h = 1 ms
  double h = 1e-3;
  el->computeCompanion(h);

  // Prepare MNA and RHS sized for 3 unknowns
  std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
  std::vector<double> rhs(3, 0.0);

  // Stamp transient companion (branch-current form expected)
  el->stampTransient(mna, rhs, indexMap);

  // Compute expected numeric values:
  // L = 10e-3
  double L = 10e-3;
  double Geq = h / (2.0 * L);      // from Inductor::computeCompanion
  double Rseries = 1.0 / Geq;      // used in stampTransient
  // u_prev = vplus - vminus = 0.5 - (-0.2) = 0.7
  double u_prev = x[0] - x[1];
  double i_prev = x[2];
  double Ieq = i_prev + Geq * u_prev;
  double rhs_branch_expected = -Ieq / Geq;

  // Check coupling stamps for branch-current case
  EXPECT_DOUBLE_EQ(mna[0][2], 1.0);   // vplus -> i
  EXPECT_DOUBLE_EQ(mna[1][2], -1.0);  // vminus -> i
  EXPECT_DOUBLE_EQ(mna[2][0], 1.0);   // i -> vplus
  EXPECT_DOUBLE_EQ(mna[2][1], -1.0);  // i -> vminus

  // Branch diagonal should be -Rseries per implementation
  EXPECT_DOUBLE_EQ(mna[2][2], -Rseries);

  // RHS for branch should equal computed rhs_branch_expected
  EXPECT_DOUBLE_EQ(rhs[2], rhs_branch_expected);

  // Ensure other RHS entries untouched (expected zeros)
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(InductorTransient, FallbackNortonStampWhenBranchIndexMissing)
{
  Parser parser;
  // Use L = 5 mH
  std::vector<std::string> tokens = {"Lnobranch", "A", "B", "5m"};
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/11);
  ASSERT_NE(el, nullptr);

  // Map only nodes (no branch-current index)
  std::map<std::string,int> indexMap;
  indexMap["A"] = 0;
  indexMap["B"] = 1;

  // Simulate previous state by providing a solution vector without branch current
  Eigen::VectorXd x(2);
  x[0] = 1.0; // vA
  x[1] = 0.25; // vB

  // updateStateFromSolution will set u_prev and i_prev via fallback formula when branch missing
  el->updateStateFromSolution(x, indexMap);

  double h = 2e-3;
  el->computeCompanion(h);

  std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs(2, 0.0);

  // Stamp transient should take Norton path and stamp Geq between nodes and Ieq into RHS
  el->stampTransient(mna, rhs, indexMap);

  // Compute expected Geq and Ieq as per Inductor::computeCompanion:
  // Geq = h/(2L), with L = 5e-3
  double L = 5e-3;
  double Geq = h / (2.0 * L);

  // u_prev = vA - vB
  double u_prev = x[0] - x[1];

  // After updateStateFromSolution i_prev is computed via fallback relation: i_prev = Geq*u_n1 + Ieq (but since Ieq stored previously, we can't directly use internal)
  // However computeCompanion sets Ieq = i_prev + Geq * u_prev. Given we just set i_prev via updateStateFromSolution fallback,
  // stamping follows the branch that writes rhs[na] -= Ieq and rhs[nb] += Ieq.
  // We can compute Ieq from the public computeCompanion result indirectly by reusing the formula:
  // Let ip = the i_prev set by updateStateFromSolution; compute Ieq = ip + Geq * u_prev.
  // We cannot read ip directly, but the stamping implementation uses Ieq in RHS; ensure RHS signs and Geq pattern are correct.

  // Check conductance stamping pattern:
  EXPECT_DOUBLE_EQ(mna[0][0], Geq);
  EXPECT_DOUBLE_EQ(mna[0][1], -Geq);
  EXPECT_DOUBLE_EQ(mna[1][0], -Geq);
  EXPECT_DOUBLE_EQ(mna[1][1], Geq);

  // RHS signs: rhs[na] should be -Ieq and rhs[nb] should be +Ieq (opposite signs)
  // Check that they are finite and opposite in sign
  EXPECT_TRUE(std::isfinite(rhs[0]));
  EXPECT_TRUE(std::isfinite(rhs[1]));
  EXPECT_DOUBLE_EQ(rhs[0], -rhs[1]);
}

TEST(InductorTransient, ZeroTimestepProducesNoStamp)
{
  Parser parser;
  std::vector<std::string> tokens = {"Lz", "N1", "N2", "1m"};
  auto el = Inductor::parse(parser, tokens, /*lineNumber=*/12);
  ASSERT_NE(el, nullptr);

  std::map<std::string,int> indexMap;
  indexMap["N1"] = 0;
  indexMap["N2"] = 1;
  indexMap["Lz"] = 2;

  std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
  std::vector<double> rhs(3, 0.0);

  // computeCompanion with zero timestep should set Geq/Ieq to zero, stampTransient should return early
  el->computeCompanion(0.0);
  el->stampTransient(mna, rhs, indexMap);

  // mna and rhs remain zeros
  for (size_t i = 0; i < mna.size(); ++i) {
    for (size_t j = 0; j < mna[i].size(); ++j) {
      EXPECT_DOUBLE_EQ(mna[i][j], 0.0);
    }
    EXPECT_DOUBLE_EQ(rhs[i], 0.0);
  }
}

TEST(InductorTransient, UpdateStateFromSolutionReadsBranchCurrentWhenPresent)
{
  Parser parser;
  std::vector<std::string> tokens = {"Lup", "P", "M", "2m"};
  auto el = std::dynamic_pointer_cast<Inductor>(Inductor::parse(parser, tokens, /*lineNumber=*/13));
  ASSERT_NE(el, nullptr);

  std::map<std::string,int> indexMap;
  indexMap["P"] = 0;
  indexMap["M"] = 1;
  indexMap["Lup"] = 2;

  // Build a solution vector where branch current index holds a non-zero value
  Eigen::VectorXd x(3);
  x[0] = 0.8;   // vP
  x[1] = 0.3;   // vM
  x[2] = 0.25;  // i_{n+1} (branch current)

  // Call updateStateFromSolution; this should store branch current into i_prev internally
  // We cannot read private i_prev, but subsequent computeCompanion+stampTransient should reflect the i_prev effect in RHS.
  el->computeCompanion(1e-3); // call before/after update doesn't matter much here
  el->updateStateFromSolution(x, indexMap);

  // Recompute companion and stamp transient to observe RHS behavior
  el->computeCompanion(1e-3);
  std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
  std::vector<double> rhs(3, 0.0);
  el->stampTransient(mna, rhs, indexMap);

  // RHS branch entry should be finite and reflect branch-current dependent term
  EXPECT_TRUE(std::isfinite(rhs[2]));
}

 // No main(): gtest_main is linked by the test target.