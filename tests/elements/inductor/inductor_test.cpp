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

#include "Inductor.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "Parser.hpp"

TEST(InductorParse, ValidToGround)
{
    Parser parser;
    std::vector<std::string> tokens = {"L1", "1", "0", "10m"};
    auto el = Inductor::parse(parser, tokens, 1);
    ASSERT_NE(el, nullptr);
    EXPECT_EQ(el->getName(), "L1");
    EXPECT_EQ(el->getNodeA(), "1");
    EXPECT_EQ(el->getNodeB(), "0");
    EXPECT_EQ(el->getGroup(), Group::G2);
}

TEST(InductorParse, InvalidTokenCount)
{
    Parser parser;
    std::vector<std::string> tokens = {"L1", "1", "0"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 2);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid inductor definition"), std::string::npos);
}

TEST(InductorParse, SameNodesRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"LBAD", "N1", "N1", "1m"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 3);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Inductor nodes cannot be the same"), std::string::npos);
}

TEST(InductorParse, IllegalZeroValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"LZ", "N1", "N2", "0"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 4);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for inductor value"),
              std::string::npos);
}

TEST(InductorParse, ZeroWithSuffixRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"LZK", "N1", "N2", "0K"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 5);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for inductor value"),
              std::string::npos);
}

TEST(InductorParse, MalformedNumericValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"LX", "N1", "N2", "1.2.3"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 6);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(InductorParse, UnknownSuffix)
{
    Parser parser;
    std::vector<std::string> tokens = {"LQ", "N1", "0", "1Q"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 7);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Unknown suffix"), std::string::npos);
}

TEST(InductorParse, EmptyMantissaRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"LK", "N1", "N2", "K"};
    testing::internal::CaptureStderr();
    auto el = Inductor::parse(parser, tokens, 8);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid value"), std::string::npos);
}

TEST(InductorTransient, ComputeCompanionAndStamp_WithBranchIndex)
{
    Parser parser;
    std::vector<std::string> tokens = {"Lbranch", "Vp", "Vm", "10m"};
    auto el = std::dynamic_pointer_cast<Inductor>(
        Inductor::parse(parser, tokens, 10));
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["Vp"] = 0;
    indexMap["Vm"] = 1;
    indexMap["Lbranch"] = 2;

    Eigen::VectorXd x(3);
    x[0] = 0.5;
    x[1] = -0.2;
    x[2] = 0.1;

    el->updateStateFromSolution(x, indexMap);

    double h = 1e-3;
    el->computeCompanion(h);

    std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
    std::vector<double> rhs(3, 0.0);

    el->stampTransient(mna, rhs, indexMap);

    double L = 10e-3;
    double Geq = h / (2.0 * L);
    double Rseries = 1.0 / Geq;
    double u_prev = x[0] - x[1];
    double i_prev = x[2];
    double Ieq = i_prev + Geq * u_prev;
    double rhs_branch_expected = -Ieq / Geq;

    EXPECT_DOUBLE_EQ(mna[0][2], 1.0);
    EXPECT_DOUBLE_EQ(mna[1][2], -1.0);
    EXPECT_DOUBLE_EQ(mna[2][0], 1.0);
    EXPECT_DOUBLE_EQ(mna[2][1], -1.0);
    EXPECT_DOUBLE_EQ(mna[2][2], -Rseries);
    EXPECT_DOUBLE_EQ(rhs[2], rhs_branch_expected);
    EXPECT_DOUBLE_EQ(rhs[0], 0.0);
    EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}

TEST(InductorTransient, FallbackNortonStampWhenBranchIndexMissing)
{
    Parser parser;
    std::vector<std::string> tokens = {"Lnobranch", "A", "B", "5m"};
    auto el = Inductor::parse(parser, tokens, 11);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["A"] = 0;
    indexMap["B"] = 1;

    Eigen::VectorXd x(2);
    x[0] = 1.0;
    x[1] = 0.25;

    el->updateStateFromSolution(x, indexMap);

    double h = 2e-3;
    el->computeCompanion(h);

    std::vector<std::vector<double>> mna(2, std::vector<double>(2, 0.0));
    std::vector<double> rhs(2, 0.0);

    el->stampTransient(mna, rhs, indexMap);

    double L = 5e-3;
    double Geq = h / (2.0 * L);

    double u_prev = x[0] - x[1];

    EXPECT_DOUBLE_EQ(mna[0][0], Geq);
    EXPECT_DOUBLE_EQ(mna[0][1], -Geq);
    EXPECT_DOUBLE_EQ(mna[1][0], -Geq);
    EXPECT_DOUBLE_EQ(mna[1][1], Geq);

    EXPECT_TRUE(std::isfinite(rhs[0]));
    EXPECT_TRUE(std::isfinite(rhs[1]));
    EXPECT_DOUBLE_EQ(rhs[0], -rhs[1]);
}

TEST(InductorTransient, ZeroTimestepProducesNoStamp)
{
    Parser parser;
    std::vector<std::string> tokens = {"Lz", "N1", "N2", "1m"};
    auto el = Inductor::parse(parser, tokens, 12);
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["N2"] = 1;
    indexMap["Lz"] = 2;

    std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
    std::vector<double> rhs(3, 0.0);

    el->computeCompanion(0.0);
    el->stampTransient(mna, rhs, indexMap);

    for (size_t i = 0; i < mna.size(); ++i) {
        for (size_t j = 0; j < mna[i].size(); ++j) {
            EXPECT_DOUBLE_EQ(mna[i][j], 0.0);
        }
        EXPECT_DOUBLE_EQ(rhs[i], 0.0);
    }
}

TEST(InductorTransient, UpdateStateFromSolutionReadsBranchCurrentWhenPresent)
{
    Parser parser;
    std::vector<std::string> tokens = {"Lup", "P", "M", "2m"};
    auto el = std::dynamic_pointer_cast<Inductor>(
        Inductor::parse(parser, tokens, 13));
    ASSERT_NE(el, nullptr);

    std::map<std::string, int> indexMap;
    indexMap["P"] = 0;
    indexMap["M"] = 1;
    indexMap["Lup"] = 2;

    Eigen::VectorXd x(3);
    x[0] = 0.8;
    x[1] = 0.3;
    x[2] = 0.25;

    el->computeCompanion(1e-3);
    el->updateStateFromSolution(x, indexMap);

    el->computeCompanion(1e-3);
    std::vector<std::vector<double>> mna(3, std::vector<double>(3, 0.0));
    std::vector<double> rhs(3, 0.0);
    el->stampTransient(mna, rhs, indexMap);

    EXPECT_TRUE(std::isfinite(rhs[2]));
}
