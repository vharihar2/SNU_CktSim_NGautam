#include <gtest/gtest.h>

#include "Parser.hpp"

#include <string>
#include <vector>

// Tests for Parser::validateTokens
// The function signature:
//   bool validateTokens(const std::vector<std::string>& tokens,
//                       int expectedSize, int lineNumber);
//
// Behavior observed from implementation:
// - returns true when tokens.size() == expectedSize
// - returns false and writes an error to std::cerr when sizes differ,
//   formatted as: "Line <lineNumber>: Expected <expectedSize> tokens, got <actual>"
//
// These tests verify the return value, the stderr output, and that the
// input vector is not modified by the call.

TEST(ValidateTokens, MatchesSizeReturnsTrueAndNoStderr) {
  Parser parser;
  std::vector<std::string> tokens = {"A", "B", "C"};
  // capture stderr to ensure nothing is printed
  testing::internal::CaptureStderr();
  bool ok = parser.validateTokens(tokens, 3, 10);
  std::string err = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(ok);
  EXPECT_TRUE(err.empty());
}

TEST(ValidateTokens, EmptyTokensExpectedZeroReturnsTrue) {
  Parser parser;
  std::vector<std::string> tokens;  // empty
  testing::internal::CaptureStderr();
  bool ok = parser.validateTokens(tokens, 0, 5);
  std::string err = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(ok);
  EXPECT_TRUE(err.empty());
}

TEST(ValidateTokens, MismatchPrintsErrorAndReturnsFalse) {
  Parser parser;
  std::vector<std::string> tokens = {"X", "Y"};
  testing::internal::CaptureStderr();
  bool ok = parser.validateTokens(tokens, 3, 42);
  std::string err = testing::internal::GetCapturedStderr();

  EXPECT_FALSE(ok);
  // Expect the error message to contain the line number, expected and actual counts.
  EXPECT_NE(err.find("Line 42:"), std::string::npos);
  EXPECT_NE(err.find("Expected 3 tokens, got 2"), std::string::npos);
}

TEST(ValidateTokens, NegativeExpectedSizeHandledAndReported) {
  Parser parser;
  std::vector<std::string> tokens = {"only"};
  testing::internal::CaptureStderr();
  bool ok = parser.validateTokens(tokens, -1, 7);
  std::string err = testing::internal::GetCapturedStderr();

  // Implementation doesn't special-case negative expectedSize; it will detect
  // mismatch and report it. We assert that the function returns false and the
  // message contains the negative expectedSize string.
  EXPECT_FALSE(ok);
  EXPECT_NE(err.find("Line 7:"), std::string::npos);
  EXPECT_NE(err.find("Expected -1 tokens, got 1"), std::string::npos);
}

TEST(ValidateTokens, DoesNotModifyInputVectorOnSuccessOrFailure) {
  Parser parser;
  std::vector<std::string> tokens = {"one", "two"};
  std::vector<std::string> before = tokens;

  // Case: success
  bool ok1 = parser.validateTokens(tokens, 2, 1);
  EXPECT_TRUE(ok1);
  EXPECT_EQ(tokens, before);  // unchanged

  // Case: failure
  bool ok2 = parser.validateTokens(tokens, 3, 2);
  EXPECT_FALSE(ok2);
  EXPECT_EQ(tokens, before);  // unchanged
}

// No main here; test binary links with gtest_main which provides main().