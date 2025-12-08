#include <gtest/gtest.h>

#include "VoltageSource.hpp"
#include "Parser.hpp"

#include <string>
#include <vector>

/*
 * voltage_test.cpp
 *
 * Unit tests for VoltageSource::parse(...).
 *
 * These tests cover the parser-side validation paths in
 * VoltageSource::parse:
 *  - valid voltage source to ground
 *  - invalid token count
 *  - invalid (same) nodes
 *  - illegal zero value (value == 0)
 *  - malformed numeric value rejected by Parser::parseValue
 *  - valid suffix/scientific combinations accepted
 *  - additional edge cases: unknown suffix, zero with suffix, empty mantissa
 *  - stamping behavior for positive and negative values
 */

TEST(VoltageParse, ValidVoltageToGround)
{
  Parser parser;
  std::vector<std::string> tokens = {"V1", "1", "0", "5"};
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  EXPECT_EQ(el->getName(), "V1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
  EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(VoltageParse, InvalidTokenCount)
{
  Parser parser;
  std::vector<std::string> tokens = {"V1", "1", "0"}; // missing value token
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid voltage source definition"), std::string::npos);
}

TEST(VoltageParse, InvalidSameNodes)
{
  Parser parser;
  std::vector<std::string> tokens = {"Vbad", "N1", "N1", "5"};
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Voltage source nodes cannot be the same"), std::string::npos);
}

TEST(VoltageParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"V0", "N1", "N2", "0"};
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/4);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for voltage source value"), std::string::npos);
}

TEST(VoltageParse, MalformedNumericValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"VX", "N1", "N2", "1.2.3"};
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // parseValue emits a diagnostic like "Line 5: Invalid value '...'"
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(VoltageParse, AcceptsSuffixAndScientific)
{
  Parser parser;
  // MEG suffix
  std::vector<std::string> tokens1 = {"VMEG", "N1", "0", "2MEG"};
  auto el1 = VoltageSource::parse(parser, tokens1, /*lineNumber=*/6);
  ASSERT_NE(el1, nullptr);

  // scientific mantissa with suffix (1e-3 * 1e3 = 1)
  std::vector<std::string> tokens2 = {"V3", "N1", "N2", "1e-3K"};
  auto el2 = VoltageSource::parse(parser, tokens2, /*lineNumber=*/7);
  ASSERT_NE(el2, nullptr);
}

TEST(VoltageParse, UnknownSuffixIsRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"VQ", "N1", "0", "1Q"}; // 'Q' unknown suffix
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/8);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // The parser should either report an unknown suffix or ultimately report
  // an illegal argument for the voltage value; check for one of those markers.
  EXPECT_TRUE(err.find("Unknown suffix") != std::string::npos ||
              err.find("Illegal argument for voltage source value") != std::string::npos);
}

TEST(VoltageParse, ZeroWithSuffixRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"VZK", "N1", "N2", "0K"}; // zero scaled -> zero
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/9);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for voltage source value"), std::string::npos);
}

TEST(VoltageParse, EmptyMantissaRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"VK", "N1", "N2", "K"}; // empty mantissa with valid suffix
  testing::internal::CaptureStderr();
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/10);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  // parseValue emits an "Invalid value" diagnostic for empty mantissa.
  EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(VoltageStamp, ToGround_PositiveValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"V1", "1", "0", "5"}; // value = 5
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/11);
  ASSERT_NE(el, nullptr);

  std::map<std::string, int> indexMap;
  indexMap["1"] = 0;
  indexMap["V1"] = 1;

  std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
  std::vector<double> rhs(2, 0.0);

  el->stamp(mna, rhs, indexMap);

  // For nodeA='1', nodeB='0' branch:
  EXPECT_DOUBLE_EQ(mna[0][1], 1.0);
  EXPECT_DOUBLE_EQ(mna[1][0], 1.0);
  EXPECT_DOUBLE_EQ(rhs[1], 5.0);
}

TEST(VoltageStamp, BetweenNodes_NegativeValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"VNEG", "N1", "N2", "-2"}; // value = -2
  auto el = VoltageSource::parse(parser, tokens, /*lineNumber=*/12);
  ASSERT_NE(el, nullptr);

  std::map<std::string, int> indexMap;
  indexMap["N1"] = 0;
  indexMap["N2"] = 1;
  indexMap["VNEG"] = 2;

  std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
  std::vector<double> rhs(3, 0.0);

  el->stamp(mna, rhs, indexMap);

  // Expected stamp for voltage source between N1 (vplus) and N2 (vminus)
  EXPECT_DOUBLE_EQ(mna[0][2], 1.0);   // vplus -> i
  EXPECT_DOUBLE_EQ(mna[1][2], -1.0);  // vminus -> i
  EXPECT_DOUBLE_EQ(mna[2][0], 1.0);   // i -> vplus
  EXPECT_DOUBLE_EQ(mna[2][1], -1.0);  // i -> vminus
  EXPECT_DOUBLE_EQ(rhs[2], -2.0);
}

// No main(): gtest_main is linked by the test target.