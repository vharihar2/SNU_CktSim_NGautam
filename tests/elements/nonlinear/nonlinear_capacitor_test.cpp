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
#include <vector>

#include "../lib/external/Eigen/Dense"
#include "NonlinearCapacitor.hpp"
#include "NonlinearModel.hpp"

static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
    return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

TEST(PolynomialChargeModel, EvaluationAndDerivativeSmallPolynomial)
{
    std::vector<double> coeffs = {0.0, 1e-6, 1e-9};
    auto model = makePolynomialChargeModel(coeffs);

    double u = 2.0;
    double q = model->q(u);
    double dqdu = model->dqdu(u);

    EXPECT_NEAR(q, 2e-6 + 4e-9, 1e-12);
    EXPECT_NEAR(dqdu, 1e-6 + 4e-9, 1e-15);
}

TEST(NonlinearCapacitorIter, ComputeCompanionIterAndStampTransient)
{
    std::vector<double> coeffs = {0.0, 1e-6, 1e-8};
    auto model = makePolynomialChargeModel(coeffs);

    auto el =
        std::make_shared<NonlinearCapacitor>("CNL", "A", "B", 1e-6, model);

    std::map<std::string, int> indexMap;
    indexMap["A"] = 0;
    indexMap["B"] = 1;

    Eigen::VectorXd xk(2);
    xk[0] = 1.0;
    xk[1] = 0.2;

    double h = 1e-3;

    el->computeCompanionIter(h, xk, indexMap);

    auto mna = makeZeroMNA(2);
    std::vector<double> rhs(2, 0.0);
    el->stampTransient(mna, rhs, indexMap);

    EXPECT_NE(mna[0][0], 0.0);
    EXPECT_NE(mna[1][1], 0.0);
    EXPECT_NE(mna[0][1], 0.0);
    EXPECT_NE(mna[1][0], 0.0);

    EXPECT_TRUE(std::isfinite(rhs[0]));
    EXPECT_TRUE(std::isfinite(rhs[1]));
}

TEST(NonlinearCapacitorState, SnapshotRestoreAndUpdateStateFromSolution)
{
    std::vector<double> coeffs = {0.0, 1e-6};
    auto model = makePolynomialChargeModel(coeffs);
    auto el =
        std::make_shared<NonlinearCapacitor>("Csave", "1", "0", 1e-6, model);

    std::map<std::string, int> idx;
    idx["1"] = 0;
    Eigen::VectorXd x(1);
    x[0] = 3.0;

    el->computeCompanion(1e-3);
    el->updateStateFromSolution(x, idx);

    auto s = el->snapshotState();
    auto el2 =
        std::make_shared<NonlinearCapacitor>("Crestore", "1", "0", 1e-6, model);
    el2->restoreState(s);

    std::vector<std::vector<double>> mna = makeZeroMNA(1);
    std::vector<double> rhs(1, 0.0);
    el2->computeCompanion(1e-3);
    el2->stampTransient(mna, rhs, idx);

    SUCCEED();
}

TEST(NonlinearCapacitor, DerivativeClampingAtMinDeriv)
{
    auto model = makePolynomialChargeModel(std::vector<double>{1.0});
    auto cap =
        std::make_shared<NonlinearCapacitor>("C_clamp", "P", "N", 1.0, model);

    std::map<std::string, int> indexMap;
    indexMap["P"] = 0;
    indexMap["N"] = 1;

    Eigen::VectorXd xk(2);
    xk[0] = 0.2;
    xk[1] = 0.1;

    double h = 1e-3;

    cap->computeCompanionIter(h, xk, indexMap);

    auto mna = makeZeroMNA(2);
    std::vector<double> rhs(2, 0.0);

    cap->stampTransient(mna, rhs, indexMap);

    const double minDeriv = 1e-12;
    double expectedGeq = 2.0 * minDeriv / h;

    EXPECT_NEAR(mna[0][0], expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
    EXPECT_NEAR(mna[1][1], expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
    EXPECT_NEAR(mna[0][1], -expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
    EXPECT_NEAR(mna[1][0], -expectedGeq, std::abs(expectedGeq) * 1e-6 + 1e-18);
}

TEST(NonlinearCapacitor, RejectsTooLargeGeqAndAvoidsStamping)
{
    double largeA1 = 1e9;
    auto model = makePolynomialChargeModel(std::vector<double>{0.0, largeA1});
    auto cap =
        std::make_shared<NonlinearCapacitor>("C_bad", "P", "N", 1.0, model);

    std::map<std::string, int> indexMap;
    indexMap["P"] = 0;
    indexMap["N"] = 1;

    Eigen::VectorXd xk(2);
    xk[0] = 1.0;
    xk[1] = 0.0;

    double h = 1e-3;

    cap->computeCompanionIter(h, xk, indexMap);

    auto mna = makeZeroMNA(2);
    std::vector<double> rhs(2, 0.0);
    cap->stampTransient(mna, rhs, indexMap);

    EXPECT_DOUBLE_EQ(mna[0][0], 0.0);
    EXPECT_DOUBLE_EQ(mna[0][1], 0.0);
    EXPECT_DOUBLE_EQ(mna[1][0], 0.0);
    EXPECT_DOUBLE_EQ(mna[1][1], 0.0);
    EXPECT_DOUBLE_EQ(rhs[0], 0.0);
    EXPECT_DOUBLE_EQ(rhs[1], 0.0);
}
