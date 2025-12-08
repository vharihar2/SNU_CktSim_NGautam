#include <gtest/gtest.h>

#include "Resistor.hpp"
#include "Parser.hpp"

#include <map>
#include <string>
#include <vector>

/*
 * resistor_test.cpp
 *
 * Unit tests for Resistor::parse(...) and Resistor::stamp(...).
 *
 * These tests exercise:
 *  - successful parsing of a resistor to ground
 *  - parsing rejection when both nodes are the same
 *  - parsing rejection when the resistor value is illegal (zero)
 *  - MNA stamping for Group-2 (branch-current) resistors to ground
 *  - MNA stamping for Group-2 resistors between two non-ground nodes
 *
 * The tests use the public interfaces defined in include/ (Parser.hpp /
 * Resistor.hpp) and rely on the test target being configured with the
 * project include directories.
 */

TEST(ResistorParse, ValidResistorToGround)
{
  Parser parser;
  std::vector<std::string> tokens = {"R1", "1", "0", "1K"};
  auto el = Resistor::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  EXPECT_EQ(el->getName(), "R1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
  // Group should be set to G2 by the Resistor parser implementation
  EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(ResistorParse, InvalidSameNodes)
{
  Parser parser;
  std::vector<std::string> tokens = {"RBAD", "N1", "N1", "10"};
  testing::internal::CaptureStderr();
  auto el = Resistor::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // The parser prints an error stating nodes cannot be the same
  EXPECT_NE(err.find("NodeA and NodeB cannot be the same"), std::string::npos);
}

TEST(ResistorParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"RZ", "N1", "N2", "0"};
  testing::internal::CaptureStderr();
  auto el = Resistor::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // Expect an error message about illegal argument for resistor value
  EXPECT_NE(err.find("Illegal argument for resistor value"), std::string::npos);
}

TEST(ResistorStamp, Group2_ToGround)
{
  Parser parser;
  std::vector<std::string> tokens = {"R1", "1", "0", "1"}; // value = 1
  auto el = Resistor::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);

  // indexMap: node '1' -> 0, branch current 'R1' -> 1
  std::map<std::string, int> indexMap;
  indexMap["1"] = 0;
  indexMap["R1"] = 1;

  std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs(2, 0.0);

  // stamp should modify mna according to the Group-2 resistor stamp
  el->stamp(mna, rhs, indexMap);

  // For nodeA == '1' and nodeB == '0' we expect:
  // mna[vplus][i] += 1.0
  // mna[i][vplus] += 1.0
  // mna[i][i] -= value (value == 1)
  EXPECT_DOUBLE_EQ(mna[0][1], 1.0);
  EXPECT_DOUBLE_EQ(mna[1][0], 1.0);
  EXPECT_DOUBLE_EQ(mna[1][1], -1.0);

  // RHS should remain zero for pure resistor stamp in this convention
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(ResistorStamp, Group2_BetweenNodes)
{
  Parser parser;
  std::vector<std::string> tokens = {"R2", "N1", "N2", "2"}; // value = 2
  auto el = Resistor::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);

  // indexMap: N1 -> 0, N2 -> 1, R2 -> 2
  std::map<std::string, int> indexMap;
  indexMap["N1"] = 0;
  indexMap["N2"] = 1;
  indexMap["R2"] = 2;

  std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
  std::vector<double> rhs(3, 0.0);

  el->stamp(mna, rhs, indexMap);

  // Expected Group-2 stamping pattern for R between N1 (vplus) and N2 (vminus):
  // mna[vplus][i] += 1.0
  // mna[vminus][i] += -1.0
  // mna[i][vplus] += 1.0
  // mna[i][vminus] += -1.0
  // mna[i][i] -= value
  EXPECT_DOUBLE_EQ(mna[0][2], 1.0);   // vplus->i
  EXPECT_DOUBLE_EQ(mna[1][2], -1.0);  // vminus->i
  EXPECT_DOUBLE_EQ(mna[2][0], 1.0);   // i->vplus
  EXPECT_DOUBLE_EQ(mna[2][1], -1.0);  // i->vminus
  EXPECT_DOUBLE_EQ(mna[2][2], -2.0);  // i diagonal (=-value)

  // RHS should remain zeros
  EXPECT_DOUBLE_EQ(rhs[0], 0.0);
  EXPECT_DOUBLE_EQ(rhs[1], 0.0);
  EXPECT_DOUBLE_EQ(rhs[2], 0.0);
}
