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

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "../lib/external/Eigen/Dense"
#include "NonlinearInductor.hpp"
#include "NonlinearModel.hpp"

static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
    return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

TEST(PolynomialFluxModel, EvalPhiAndDerivative)
{
    std::vector<double> coeffs = {0.0, 1e-3, 1e-4};
    auto model = makePolynomialFluxModel(coeffs);

    double i = 0.2;
    double phi = model->phi(i);
    double dphi = model->dphidi(i);

    EXPECT_NEAR(phi, 2.04e-4, 1e-12);
    EXPECT_NEAR(dphi, 0.00104, 1e-12);
}

TEST(NonlinearInductor, BranchCompanionIterAndStampTransient)
{
    double L = 10e-3;
    auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
    auto ind = std::make_shared<NonlinearInductor>("L_lin", "P", "N", L, model);

    std::map<std::string, int> indexMap;
    indexMap["P"] = 0;
    indexMap["N"] = 1;
    indexMap["L_lin"] = 2;

    Eigen::VectorXd xk(3);
    xk[0] = 0.5;
    xk[1] = -0.2;
    xk[2] = 0.05;

    double h = 1e-3;

    ind->computeCompanionIter(h, xk, indexMap);

    auto mna = makeZeroMNA(3);
    std::vector<double> rhs(3, 0.0);

    ind->stampTransient(mna, rhs, indexMap);

    double expectedGeq = h / (2.0 * L);
    double expectedRseries = 1.0 / expectedGeq;

    EXPECT_DOUBLE_EQ(mna[0][2], 1.0);
    EXPECT_DOUBLE_EQ(mna[1][2], -1.0);
    EXPECT_DOUBLE_EQ(mna[2][0], 1.0);
    EXPECT_DOUBLE_EQ(mna[2][1], -1.0);

    EXPECT_DOUBLE_EQ(mna[2][2], -expectedRseries);
    EXPECT_TRUE(std::isfinite(rhs[2]));
}

TEST(NonlinearInductor, FallbackNodeBasedStampingWhenBranchMissing)
{
    double L = 5e-3;
    auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
    auto ind = std::make_shared<NonlinearInductor>("L_nb", "A", "B", L, model);

    std::map<std::string, int> indexMap;
    indexMap["A"] = 0;
    indexMap["B"] = 1;

    Eigen::VectorXd xk(2);
    xk[0] = 1.0;
    xk[1] = 0.25;

    double h = 2e-3;
    ind->computeCompanionIter(h, xk, indexMap);

    auto mna = makeZeroMNA(2);
    std::vector<double> rhs(2, 0.0);
    ind->stampTransient(mna, rhs, indexMap);

    EXPECT_NE(mna[0][0], 0.0);
    EXPECT_NE(mna[0][1], 0.0);
    EXPECT_NE(mna[1][0], 0.0);
    EXPECT_NE(mna[1][1], 0.0);

    EXPECT_TRUE(std::isfinite(rhs[0]));
    EXPECT_TRUE(std::isfinite(rhs[1]));
    EXPECT_DOUBLE_EQ(rhs[0], -rhs[1]);
}

TEST(NonlinearInductor,
     UpdateStateReadsBranchCurrentWhenPresentAndSnapshotRestore)
{
    double L = 2e-3;
    auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
    auto ind = std::make_shared<NonlinearInductor>("L_up", "P", "M", L, model);

    std::map<std::string, int> indexMap;
    indexMap["P"] = 0;
    indexMap["M"] = 1;
    indexMap["L_up"] = 2;

    Eigen::VectorXd x(3);
    x[0] = 0.8;
    x[1] = 0.3;
    x[2] = 0.123;

    ind->updateStateFromSolution(x, indexMap);

    auto snap = ind->snapshotState();
    auto ind2 =
        std::make_shared<NonlinearInductor>("L_rest", "P", "M", L, model);
    ind2->restoreState(snap);

    ind2->computeCompanion(1e-3);
    auto mna = makeZeroMNA(3);
    std::vector<double> rhs(3, 0.0);
    ind2->stampTransient(mna, rhs, indexMap);

    EXPECT_TRUE(std::isfinite(rhs[2]));
    EXPECT_TRUE(std::isfinite(mna[2][2]));
}

TEST(NonlinearInductor, ComputeCompanionHandlesZeroTimestepGracefully)
{
    double L = 1e-3;
    auto model = makePolynomialFluxModel(std::vector<double>{0.0, L});
    auto ind =
        std::make_shared<NonlinearInductor>("L_zero", "X", "Y", L, model);

    std::map<std::string, int> indexMap;
    indexMap["X"] = 0;
    indexMap["Y"] = 1;
    indexMap["L_zero"] = 2;

    ind->computeCompanion(0.0);

    auto mna = makeZeroMNA(3);
    std::vector<double> rhs(3, 0.0);
    ind->stampTransient(mna, rhs, indexMap);

    for (size_t i = 0; i < mna.size(); ++i) {
        for (size_t j = 0; j < mna[i].size(); ++j) {
            EXPECT_DOUBLE_EQ(mna[i][j], 0.0);
        }
        EXPECT_DOUBLE_EQ(rhs[i], 0.0);
    }
}

// End of file - no main() here; gtest_main provides test runner.
