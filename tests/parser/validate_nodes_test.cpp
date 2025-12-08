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

#include "Parser.hpp"

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
    EXPECT_NE(err.find("NodeA and NodeB cannot be the same (NODE)"),
              std::string::npos);
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
