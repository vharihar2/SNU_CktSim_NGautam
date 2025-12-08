#include <gtest/gtest.h>

#include "Parser.hpp"
#include <string>

/*
 * validate_nodes_test.cpp
 *
 * Unit tests for Parser::validateNodes.
 *
 * Behavior:
 *   - Returns false and prints an error message when nodeA == nodeB.
 *   - Returns true when nodeA != nodeB.
 *
 * We capture stderr to verify the diagnostic message in the equality case.
 */

TEST(ValidateNodes, DifferentNodesReturnTrue)
{
  Parser parser;
  bool ok = parser.validateNodes("N1", "N2", /*lineNumber=*/10);
  EXPECT_TRUE(ok);
}

TEST(ValidateNodes, SameNodesReturnFalseAndPrintsMessage)
{
  Parser parser;
  testing::internal::CaptureStderr();
  bool ok = parser.validateNodes("NODE", "NODE", /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(ok);
  EXPECT_NE(err.find("Line 5"), std::string::npos);
  EXPECT_NE(err.find("NodeA and NodeB cannot be the same (NODE)"), std::string::npos);
}

TEST(ValidateNodes, BothEmptyStringsTreatedAsSame)
{
  Parser parser;
  testing::internal::CaptureStderr();
  bool ok = parser.validateNodes("", "", /*lineNumber=*/7);
  std::string err = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(ok);
  EXPECT_NE(err.find("Line 7"), std::string::npos);
}

TEST(ValidateNodes, OneEmptyOneNonEmptyIsValid)
{
  Parser parser;
  bool ok = parser.validateNodes("0", "", /*lineNumber=*/8);
  EXPECT_TRUE(ok);

  ok = parser.validateNodes("", "N1", /*lineNumber=*/9);
  EXPECT_TRUE(ok);
}

// No main(): test binary links with gtest_main which supplies main().