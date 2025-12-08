/*
 * Copyright (c) 2022, Shiv Nadar University, Delhi NCR, India. All Rights
 * Reserved. Permission to use, copy, modify and distribute this software for
 * educational, research, and not-for-profit purposes, without fee and without a
 * signed license agreement, is hereby granted, provided that this paragraph and
 * the following two paragraphs appear in all copies, modifications, and
 * distributions.
 *
 * IN NO EVENT SHALL SHIV NADAR UNIVERSITY BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
 * PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE.
 *
 * SHIV NADAR UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS PROVIDED "AS IS". SHIV
 * NADAR UNIVERSITY HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 */
#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "CurrentSource.hpp"
#include "Parser.hpp"

TEST(CurrentParse, ValidCurrentToGround)
{
    Parser parser;
    std::vector<std::string> tokens = {"I1", "1", "0", "2m"};
    auto el = CurrentSource::parse(parser, tokens, 1);
    ASSERT_NE(el, nullptr);
    EXPECT_EQ(el->getName(), "I1");
    EXPECT_EQ(el->getNodeA(), "1");
    EXPECT_EQ(el->getNodeB(), "0");
    EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(CurrentParse, InvalidTokenCount)
{
    Parser parser;
    std::vector<std::string> tokens = {"I1", "1", "0"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 2);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid current source definition"), std::string::npos);
}

TEST(CurrentParse, InvalidSameNodes)
{
    Parser parser;
    std::vector<std::string> tokens = {"IBAD", "N1", "N1", "10u"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 3);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Current source nodes cannot be the same"),
              std::string::npos);
}

TEST(CurrentParse, IllegalZeroValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"IZ", "N1", "N2", "0"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 4);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for current source value"),
              std::string::npos);
}

TEST(CurrentParse, ZeroAfterSuffixRejected)
{
    Parser parser;
    // 0K -> 0 after applying suffix multiplier -> still illegal
    std::vector<std::string> tokens = {"I0K", "N1", "N2", "0K"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 5);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for current source value"),
              std::string::npos);
}

TEST(CurrentParse, MalformedNumericValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"IX", "N1", "N2", "1.2.3"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 6);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    // parseValue prints an "Invalid value" diagnostic; parse() prints an
    // illegal argument message.
    EXPECT_TRUE(err.find("Invalid value") != std::string::npos ||
                err.find("Illegal argument for current source value") !=
                    std::string::npos);
}

TEST(CurrentParse, UnknownSuffixIsRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"IQ", "N1", "0", "1Q"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 7);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Unknown suffix"), std::string::npos);
}

TEST(CurrentParse, EmptyMantissaRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"IK", "N1", "N2", "K"};
    testing::internal::CaptureStderr();
    auto el = CurrentSource::parse(parser, tokens, 8);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(CurrentStamp, ToGround_RHSSignsAndValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"I1", "1", "0", "0.01"};
    auto el = CurrentSource::parse(parser, tokens, 9);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["1"] = 0;

    std::vector<std::vector<double>> mna(1, std::vector<double>(1, 0.0));
    std::vector<double> rhs(1, 0.0);

    // For current source with nodeB == 0: nodeA != 0 => rhs[vplus] -= value
    el->stamp(mna, rhs, indexMap);

    EXPECT_DOUBLE_EQ(rhs[0], -0.01);
}

TEST(CurrentStamp, BetweenNodes_RHSDistribution)
{
    Parser parser;
    std::vector<std::string> tokens = {"I2", "N1", "N2", "5m"};
    auto el = CurrentSource::parse(parser, tokens, 10);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["N2"] = 1;

    std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs(2, 0.0);

    el->stamp(mna, rhs, indexMap);

    // For nodeA != 0: rhs[vplus] -= value
    // For nodeB != 0: rhs[vminus] += value
    EXPECT_DOUBLE_EQ(rhs[0], -0.005);
    EXPECT_DOUBLE_EQ(rhs[1], 0.005);
}

TEST(CurrentStamp, NegativeValueHandledCorrectly)
{
    Parser parser;
    std::vector<std::string> tokens = {"INEG", "N1", "N2", "-2"};
    auto el = CurrentSource::parse(parser, tokens, 11);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["N2"] = 1;

    std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs(2, 0.0);

    el->stamp(mna, rhs, indexMap);

    // Negative value: rhs[N1] -= (-2) -> +2 ; rhs[N2] += (-2) -> -2
    EXPECT_DOUBLE_EQ(rhs[0], 2.0);
    EXPECT_DOUBLE_EQ(rhs[1], -2.0);
}
