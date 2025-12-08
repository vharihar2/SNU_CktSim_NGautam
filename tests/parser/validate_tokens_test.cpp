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

#include <string>
#include <vector>

#include "Parser.hpp"

TEST(ValidateTokens, MatchesSizeReturnsTrueAndNoStderr)
{
    Parser parser;
    std::vector<std::string> tokens = {"A", "B", "C"};
    testing::internal::CaptureStderr();
    bool ok = parser.validateTokens(tokens, 3, 10);
    std::string err = testing::internal::GetCapturedStderr();

    EXPECT_TRUE(ok);
    EXPECT_TRUE(err.empty());
}

TEST(ValidateTokens, EmptyTokensExpectedZeroReturnsTrue)
{
    Parser parser;
    std::vector<std::string> tokens;
    testing::internal::CaptureStderr();
    bool ok = parser.validateTokens(tokens, 0, 5);
    std::string err = testing::internal::GetCapturedStderr();

    EXPECT_TRUE(ok);
    EXPECT_TRUE(err.empty());
}

TEST(ValidateTokens, MismatchPrintsErrorAndReturnsFalse)
{
    Parser parser;
    std::vector<std::string> tokens = {"X", "Y"};
    testing::internal::CaptureStderr();
    bool ok = parser.validateTokens(tokens, 3, 42);
    std::string err = testing::internal::GetCapturedStderr();

    EXPECT_FALSE(ok);
    EXPECT_NE(err.find("Line 42:"), std::string::npos);
    EXPECT_NE(err.find("Expected 3 tokens, got 2"), std::string::npos);
}

TEST(ValidateTokens, NegativeExpectedSizeHandledAndReported)
{
    Parser parser;
    std::vector<std::string> tokens = {"only"};
    testing::internal::CaptureStderr();
    bool ok = parser.validateTokens(tokens, -1, 7);
    std::string err = testing::internal::GetCapturedStderr();

    EXPECT_FALSE(ok);
    EXPECT_NE(err.find("Line 7:"), std::string::npos);
    EXPECT_NE(err.find("Expected -1 tokens, got 1"), std::string::npos);
}

TEST(ValidateTokens, DoesNotModifyInputVectorOnSuccessOrFailure)
{
    Parser parser;
    std::vector<std::string> tokens = {"one", "two"};
    std::vector<std::string> before = tokens;

    bool ok1 = parser.validateTokens(tokens, 2, 1);
    EXPECT_TRUE(ok1);
    EXPECT_EQ(tokens, before);

    bool ok2 = parser.validateTokens(tokens, 3, 2);
    EXPECT_FALSE(ok2);
    EXPECT_EQ(tokens, before);
}
