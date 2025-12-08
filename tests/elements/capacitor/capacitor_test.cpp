#include <gtest/gtest.h>

#include "Capacitor.hpp"
#include "Parser.hpp"

#include <map>
#include <string>
#include <vector>

/*
 * capacitor_test.cpp
 *
 * Unit tests for Capacitor::parse(...), Capacitor::computeCompanion(...),
 * Capacitor::stampTransient(...), and updateStateFromSolution behavior insofar
 * as it affects observable stamping (via subsequent computeCompanion + stampTransient).
 *
 * The tests exercise:
 *  - successful parsing of a capacitor (to ground)
 *  - parsing rejection on malformed input (tokens, same nodes, zero value)
 *  - rejection of zero after suffix, unknown suffix, empty mantissa
 *  - transient companion computation with a valid timestep -> Geq applied to MNA
 *  - transient stamping behavior for node-to-ground and between-two-nodes
 *  - guard behavior when timestep <= 0 (no stamping / Geq==0)
 *
 * Notes:
 *  - Internal state members (v_prev, i_prev, Geq, Ieq) are private; tests
 *    therefore validate effects visible through public API (stampTransient,
 *    updateStateFromSolution) and documented stamping side-effects.
 */

TEST(CapacitorParse, ValidCapacitorToGround)
{
  Parser parser;
  // 1uF (1 microfarad) using 'U' suffix
  std::vector<std::string> tokens = {"C1", "1", "0", "1U"};
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  EXPECT_EQ(el->getName(), "C1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
  EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(CapacitorParse, InvalidTokenCount)
{
  Parser parser;
  std::vector<std::string> tokens = {"C1", "1", "0"}; // missing value
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid capacitor definition"), std::string::npos);
}

TEST(CapacitorParse, InvalidSameNodes)
{
  Parser parser;
  std::vector<std::string> tokens = {"CBAD", "N1", "N1", "10p"};
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Capacitor nodes cannot be the same"), std::string::npos);
}

TEST(CapacitorParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"CZ", "N1", "N2", "0"};
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/4);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for capacitor value"), std::string::npos);
}

TEST(CapacitorParse, ZeroWithSuffixRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"CZK", "N1", "N2", "0K"};
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for capacitor value"), std::string::npos);
}

TEST(CapacitorParse, MalformedNumericValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"CX", "N1", "N2", "1.2.3"};
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/6);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // parseValue emits "Invalid value" diagnostic
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(CapacitorParse, UnknownSuffix)
{
  Parser parser;
  std::vector<std::string> tokens = {"CQ", "N1", "0", "1Q"};
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/7);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Unknown suffix"), std::string::npos);
}

TEST(CapacitorParse, EmptyMantissaRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"CK", "N1", "N2", "K"}; // no mantissa
  testing::internal::CaptureStderr();
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/8);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(CapacitorTransient, ComputeCompanionAndStamp_NodeToGround)
{
  Parser parser;
  // Use 1 uF capacitor
  std::vector<std::string> tokens = {"Ctg", "Nplus", "0", "1U"};
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/10);
  ASSERT_NE(el, nullptr);

  // Prepare mapping: node Nplus -> 0
  std::map<std::string,int> indexMap;
  indexMap["Nplus"] = 0;

  // Prepare small MNA + RHS
  std::vector<std::vector<double>> mna(1, std::vector<double>(1, 0.0));
  std::vector<double> rhs(1, 0.0);

  // Compute companion with a non-zero timestep h
  double h = 1e-3; // 1 ms
  // The capacitor's computeCompanion is a member function; call it via dynamic cast
  el->computeCompanion(h);

  // Stamp transient companion into MNA/RHS
  el->stampTransient(mna, rhs, indexMap);

  // Expected Geq = 2*C / h = 2 * 1e-6 / 1e-3 = 2e-3
  double expectedGeq = 2.0 * 1e-6 / h; // = 0.002

  // For nodeA=Nplus and nodeB=0, stampTransient should add Geq to mna[0][0]
  EXPECT_DOUBLE_EQ(mna[0][0], expectedGeq);

  // With default v_prev==0 and i_prev==0, Ieq == 0 so rhs remains zero
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
}

TEST(CapacitorTransient, ComputeCompanionAndStamp_BetweenNodes)
{
  Parser parser;
  std::vector<std::string> tokens = {"Cbb", "Na", "Nb", "1u"}; // 1 uF
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/11);
  ASSERT_NE(el, nullptr);

  // Map nodes to indices
  std::map<std::string,int> indexMap;
  indexMap["Na"] = 0;
  indexMap["Nb"] = 1;

  std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs(2, 0.0);

  double h = 2e-3; // 2 ms
  el->computeCompanion(h);
  el->stampTransient(mna, rhs, indexMap);

  double expectedGeq = 2.0 * 1e-6 / h; // 2*C/h

  // Check symmetric stamping for a two-node conductance:
  EXPECT_DOUBLE_EQ(mna[0][0], expectedGeq);
  EXPECT_DOUBLE_EQ(mna[0][1], -expectedGeq);
  EXPECT_DOUBLE_EQ(mna[1][0], -expectedGeq);
  EXPECT_DOUBLE_EQ(mna[1][1], expectedGeq);

  // With v_prev and i_prev defaults, Ieq == 0 so rhs entries remain zero
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(CapacitorTransient, ZeroTimestepProducesNoStamp)
{
  Parser parser;
  std::vector<std::string> tokens = {"Cz", "N1", "N2", "1U"};
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/12);
  ASSERT_NE(el, nullptr);

  std::map<std::string,int> indexMap;
  indexMap["N1"] = 0;
  indexMap["N2"] = 1;

  std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs(2, 0.0);

  // computeCompanion with h <= 0 should set Geq/Ieq to zero and avoid stamping
  el->computeCompanion(0.0);
  el->stampTransient(mna, rhs, indexMap);

  // mna and rhs must remain unchanged (all zeros)
  EXPECT_DOUBLE_EQ(mna[0][0], 0.0);
  EXPECT_DOUBLE_EQ(mna[0][1], 0.0);
  EXPECT_DOUBLE_EQ(mna[1][0], 0.0);
  EXPECT_DOUBLE_EQ(mna[1][1], 0.0);
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(CapacitorTransient, UpdateStateAndSubsequentStampingConsistency)
{
  Parser parser;
  std::vector<std::string> tokens = {"Cup", "Vp", "Vm", "2U"}; // C = 2 uF
  auto el = Capacitor::parse(parser, tokens, /*lineNumber=*/13);
  ASSERT_NE(el, nullptr);

  std::map<std::string,int> indexMap;
  indexMap["Vp"] = 0;
  indexMap["Vm"] = 1;

  // initial companion with h1
  double h1 = 1e-3;
  el->computeCompanion(h1);

  std::vector<std::vector<double>> mna1(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs1(2, 0.0);
  el->stampTransient(mna1, rhs1, indexMap);

  // Now simulate solver producing node voltages vplus=1.2, vminus=0.2
  Eigen::VectorXd x(2);
  x[0] = 1.2;
  x[1] = 0.2;

  // Update internal state from solution (this sets v_prev and i_prev internally)
  el->updateStateFromSolution(x, indexMap);

  // Recompute companion with the same h and stamp again; stamping should be
  // well-formed (does not blow up) and RHS should reflect the updated Ieq value
  el->computeCompanion(h1);

  std::vector<std::vector<double>> mna2(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs2(2, 0.0);
  el->stampTransient(mna2, rhs2, indexMap);

  // The conductance stamping should be identical to previous pattern (2*C/h)
  double expectedGeq = 2.0 * 2e-6 / h1; // 2*C/h with C=2u
  EXPECT_DOUBLE_EQ(mna2[0][0], expectedGeq);
  EXPECT_DOUBLE_EQ(mna2[0][1], -expectedGeq);
  EXPECT_DOUBLE_EQ(mna2[1][0], -expectedGeq);
  EXPECT_DOUBLE_EQ(mna2[1][1], expectedGeq);

  // RHS will generally be non-zero after state update since Ieq may change,
  // but we cannot inspect private Ieq directly. Ensure RHS entries are finite
  // and have opposite sign (rhs[vplus] = -Ieq, rhs[vminus] = +Ieq) by checking
  // that rhs[0] == -rhs[1].
  EXPECT_TRUE(std::isfinite(rhs2[0]));
  EXPECT_TRUE(std::isfinite(rhs2[1]));
  EXPECT_DOUBLE_EQ(rhs2[0], -rhs2[1]);
}

// No main(): gtest_main is linked by the test target.