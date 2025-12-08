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
#include "Capacitor.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "Parser.hpp"

TEST(CapacitorParse, ValidCapacitorToGround)
{
    Parser parser;
    std::vector<std::string> tokens = {"C1", "1", "0", "1U"};
    auto el = Capacitor::parse(parser, tokens, 1);
    ASSERT_NE(el, nullptr);
    EXPECT_EQ(el->getName(), "C1");
    EXPECT_EQ(el->getNodeA(), "1");
    EXPECT_EQ(el->getNodeB(), "0");
    EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(CapacitorParse, InvalidTokenCount)
{
    Parser parser;
    std::vector<std::string> tokens = {"C1", "1", "0"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 2);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid capacitor definition"), std::string::npos);
}

TEST(CapacitorParse, InvalidSameNodes)
{
    Parser parser;
    std::vector<std::string> tokens = {"CBAD", "N1", "N1", "10p"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 3);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Capacitor nodes cannot be the same"),
              std::string::npos);
}

TEST(CapacitorParse, IllegalZeroValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"CZ", "N1", "N2", "0"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 4);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for capacitor value"),
              std::string::npos);
}

TEST(CapacitorParse, ZeroWithSuffixRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"CZK", "N1", "N2", "0K"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 5);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for capacitor value"),
              std::string::npos);
}

TEST(CapacitorParse, MalformedNumericValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"CX", "N1", "N2", "1.2.3"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 6);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(CapacitorParse, UnknownSuffix)
{
    Parser parser;
    std::vector<std::string> tokens = {"CQ", "N1", "0", "1Q"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 7);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Unknown suffix"), std::string::npos);
}

TEST(CapacitorParse, EmptyMantissaRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"CK", "N1", "N2", "K"};
    testing::internal::CaptureStderr();
    auto el = Capacitor::parse(parser, tokens, 8);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(CapacitorTransient, ComputeCompanionAndStamp_NodeToGround)
{
    Parser parser;
    std::vector<std::string> tokens = {"Ctg", "Nplus", "0", "1U"};
    auto el = Capacitor::parse(parser, tokens, 10);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["Nplus"] = 0;

    std::vector<std::vector<double>> mna(1, std::vector<double>(1, 0.0));
    std::vector<double> rhs(1, 0.0);

    double h = 1e-3;
    el->computeCompanion(h);
    el->stampTransient(mna, rhs, indexMap);

    double expectedGeq = 2.0 * 1e-6 / h;
    EXPECT_DOUBLE_EQ(mna[0][0], expectedGeq);
    EXPECT_DOUBLE_EQ(rhs[0], 0.0);
}

TEST(CapacitorTransient, ComputeCompanionAndStamp_BetweenNodes)
{
    Parser parser;
    std::vector<std::string> tokens = {"Cbb", "Na", "Nb", "1u"};
    auto el = Capacitor::parse(parser, tokens, 11);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["Na"] = 0;
    indexMap["Nb"] = 1;

    std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs(2, 0.0);

    double h = 2e-3;
    el->computeCompanion(h);
    el->stampTransient(mna, rhs, indexMap);

    double expectedGeq = 2.0 * 1e-6 / h;

    EXPECT_DOUBLE_EQ(mna[0][0], expectedGeq);
    EXPECT_DOUBLE_EQ(mna[0][1], -expectedGeq);
    EXPECT_DOUBLE_EQ(mna[1][0], -expectedGeq);
    EXPECT_DOUBLE_EQ(mna[1][1], expectedGeq);

    EXPECT_DOUBLE_EQ(rhs[0], 0.0);
    EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(CapacitorTransient, ZeroTimestepProducesNoStamp)
{
    Parser parser;
    std::vector<std::string> tokens = {"Cz", "N1", "N2", "1U"};
    auto el = Capacitor::parse(parser, tokens, 12);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["N2"] = 1;

    std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs(2, 0.0);

    el->computeCompanion(0.0);
    el->stampTransient(mna, rhs, indexMap);

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
    std::vector<std::string> tokens = {"Cup", "Vp", "Vm", "2U"};
    auto el = Capacitor::parse(parser, tokens, 13);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["Vp"] = 0;
    indexMap["Vm"] = 1;

    double h1 = 1e-3;
    el->computeCompanion(h1);

    std::vector<std::vector<double>> mna1(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs1(2, 0.0);
    el->stampTransient(mna1, rhs1, indexMap);

    Eigen::VectorXd x(2);
    x[0] = 1.2;
    x[1] = 0.2;

    el->updateStateFromSolution(x, indexMap);

    el->computeCompanion(h1);

    std::vector<std::vector<double>> mna2(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs2(2, 0.0);
    el->stampTransient(mna2, rhs2, indexMap);

    double expectedGeq = 2.0 * 2e-6 / h1;
    EXPECT_DOUBLE_EQ(mna2[0][0], expectedGeq);
    EXPECT_DOUBLE_EQ(mna2[0][1], -expectedGeq);
    EXPECT_DOUBLE_EQ(mna2[1][0], -expectedGeq);
    EXPECT_DOUBLE_EQ(mna2[1][1], expectedGeq);

    EXPECT_TRUE(std::isfinite(rhs2[0]));
    EXPECT_TRUE(std::isfinite(rhs2[1]));
    EXPECT_DOUBLE_EQ(rhs2[0], -rhs2[1]);
}
