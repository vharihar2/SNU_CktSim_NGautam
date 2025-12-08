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

#include <Parser.hpp>
#include <cmath>

TEST(ParseValue, EmptyInput)
{
    Parser p;
    bool ok = true;
    double v = p.parseValue("", /*lineNumber=*/1, ok);
    EXPECT_FALSE(ok);
    // On failure parser returns 0.0; ensure value is finite (not NaN).
    EXPECT_TRUE(std::isfinite(v));
}

TEST(ParseValue, PlainDecimalsAndSigns)
{
    Parser p;
    bool ok = false;

    double v1 = p.parseValue("123", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v1, 123.0);

    double v2 = p.parseValue("-12.34", 2, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v2, -12.34);

    double v3 = p.parseValue("+0.5", 3, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v3, 0.5);
}

TEST(ParseValue, ScientificLowerUpperE)
{
    Parser p;
    bool ok = false;

    double v1 = p.parseValue("1e-3", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v1, 1e-3);

    double v2 = p.parseValue("-2E6", 2, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v2, -2e6);

    double v3 = p.parseValue("3.5e2", 3, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v3, 350.0);
}

TEST(ParseValue, SingleLetterSuffixes)
{
    Parser p;
    bool ok = false;

    // 'T' -> 1e12
    double t = p.parseValue("1T", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(t, 1e12);

    // 'G' -> 1e9
    double g = p.parseValue("2G", 2, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(g, 2e9);

    // 'K' -> 1e3
    double k = p.parseValue("3K", 3, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(k, 3e3);

    // 'M' -> milli (1e-3) per parser's suffix map
    double milli = p.parseValue("4M", 4, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(milli, 4e-3);

    // 'U' -> micro
    double u = p.parseValue("5U", 5, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(u, 5e-6);

    // 'N' -> nano
    double n = p.parseValue("6N", 6, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(n, 6e-9);

    // 'P' -> pico
    double pfx = p.parseValue("7P", 7, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(pfx, 7e-12);

    // 'F' -> femto
    double f = p.parseValue("8F", 8, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(f, 8e-15);
}

TEST(ParseValue, MultiLetterSuffixMEG)
{
    Parser p;
    bool ok = false;

    double v = p.parseValue("2MEG", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v, 2e6);
}

TEST(ParseValue, LowercaseSuffixNormalization)
{
    Parser p;
    bool ok = false;

    // lowercase 'k' should be normalized to 'K'
    double v1 = p.parseValue("1k", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v1, 1e3);

    // lowercase 'meg' normalized to 'MEG'
    double v2 = p.parseValue("2meg", 2, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v2, 2e6);

    // mixed-case mantissa with suffix: mantissa may include scientific notation
    double v3 = p.parseValue("3.3e-3u", 3,
                             ok);  // mantissa '3.3e-3', suffix 'u' -> *1e-6
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v3, 3.3e-3 * 1e-6);
}

TEST(ParseValue, UnknownSuffix)
{
    Parser p;

    bool ok1 = true;
    double v1 = p.parseValue("1Q", 1, ok1);  // 'Q' not in map
    EXPECT_FALSE(ok1);

    bool ok2 = true;
    double v2 =
        p.parseValue("5FOO", 2, ok2);  // 'FOO' unknown multi-letter suffix
    EXPECT_FALSE(ok2);
}

TEST(ParseValue, InvalidMantissaNoSuffix)
{
    Parser p;

    // Completely non-numeric without suffix: parser will treat trailing alpha
    // as suffix and therefore report unknown suffix. Use a fresh flag per call.
    bool ok1 = true;
    double v1 = p.parseValue("ABC", 1, ok1);
    EXPECT_FALSE(ok1);

    // With strict parsing enabled, malformed numeric strings such as "1.2.3"
    // must be rejected (no partial-prefix acceptance).
    bool ok2 = true;
    double v2 = p.parseValue("1.2.3", 2, ok2);
    EXPECT_FALSE(ok2);
}

TEST(ParseValue, InvalidMantissaWithSuffix)
{
    Parser p;

    bool ok1 = true;
    double v1 =
        p.parseValue("ABCK", 1, ok1);  // invalid mantissa + suffix present
    EXPECT_FALSE(ok1);

    bool ok2 = true;
    double v2 = p.parseValue(
        "K", 2, ok2);  // empty mantissa with valid suffix -> invalid
    EXPECT_FALSE(ok2);
}

TEST(ParseValue, ScientificMantissaWithSuffix)
{
    Parser p;
    bool ok = false;

    double v = p.parseValue("1e-3K", 1, ok);  // 1e-3 * 1e3 = 1.0
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v, 1.0);

    double v2 = p.parseValue("-2.5e2M", 2, ok);  // -250 * 1e-3 = -0.25
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(v2, -0.25);
}

TEST(ParseValue, LargeAndSmallValues)
{
    Parser p;
    bool ok = false;

    double big = p.parseValue("1.5T", 1, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(big, 1.5e12);

    double tiny = p.parseValue("2F", 2, ok);
    EXPECT_TRUE(ok);
    EXPECT_DOUBLE_EQ(tiny, 2e-15);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
