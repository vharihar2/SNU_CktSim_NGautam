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
#include <memory>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "CurrentSource.hpp"
#include "DependentVoltageSource.hpp"
#include "Parser.hpp"
#include "VoltageSource.hpp"

class TestDependentVoltageSource : public DependentVoltageSource
{
   public:
    using DependentVoltageSource::DependentVoltageSource;

    void setControllingElementPtr(std::shared_ptr<CircuitElement> el)
    {
        this->controlling_element = std::move(el);
    }

    void setControllingVariableEnum(ControlVariable cv)
    {
        this->controlling_variable = cv;
    }
};

static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
    return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

TEST(DependentVoltageParse, ValidParseBasic)
{
    Parser parser;
    std::vector<std::string> tokens = {"VC1", "1", "0", "2", "V", "VCTRL"};
    auto el = DependentVoltageSource::parse(parser, tokens, 1);
    ASSERT_NE(el, nullptr);
    EXPECT_EQ(el->getName(), "VC1");
    EXPECT_EQ(el->getNodeA(), "1");
    EXPECT_EQ(el->getNodeB(), "0");
}

TEST(DependentVoltageParse, InvalidTokenCount)
{
    Parser parser;
    std::vector<std::string> tokens = {"VC1", "1", "0", "2", "V"};
    testing::internal::CaptureStderr();
    auto el = DependentVoltageSource::parse(parser, tokens, 2);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid dependent voltage source definition"),
              std::string::npos);
}

TEST(DependentVoltageParse, SameNodesRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"Vbad", "N1", "N1", "1", "V", "X1"};
    testing::internal::CaptureStderr();
    auto el = DependentVoltageSource::parse(parser, tokens, 3);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Dependent voltage source nodes cannot be the same"),
              std::string::npos);
}

TEST(DependentVoltageParse, IllegalZeroValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"VZ", "N1", "N2", "0", "V", "V1"};
    testing::internal::CaptureStderr();
    auto el = DependentVoltageSource::parse(parser, tokens, 4);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for dependent voltage source value"),
              std::string::npos);
}

TEST(DependentVoltageParse, IllegalControllingVariable)
{
    Parser parser;
    std::vector<std::string> tokens = {"Vx", "A", "B", "1", "Q", "X1"};
    testing::internal::CaptureStderr();
    auto el = DependentVoltageSource::parse(parser, tokens, 5);
    std::string err = testing::internal::GetCapturedStderr();
    ASSERT_NE(el, nullptr);
    EXPECT_NE(err.find("Illegal controlling variable argument"),
              std::string::npos);
}

TEST(DependentVoltageParse, CascadingControlledSourceRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"VE", "1", "0",
                                       "5",  "V", "VC_SOMETHING"};
    testing::internal::CaptureStderr();
    auto el = DependentVoltageSource::parse(parser, tokens, 6);
    std::string err = testing::internal::GetCapturedStderr();
    ASSERT_NE(el, nullptr);
    EXPECT_NE(err.find("cannot be cascaded"), std::string::npos);
}

TEST(DependentVoltageStamp, VCVS_BetweenNodes_UsesControllingNodes)
{
    auto ctrl = std::make_shared<VoltageSource>("VCTRL", "X", "Y", 5.0);
    auto ev = std::make_shared<TestDependentVoltageSource>("E1", "A", "B", 2.0);
    ev->setControllingVariableEnum(ControlVariable::v);
    ev->setControllingElementPtr(ctrl);

    std::map<std::string, int> indexMap;
    indexMap["A"] = 0;
    indexMap["B"] = 1;
    indexMap["X"] = 2;
    indexMap["Y"] = 3;
    indexMap["E1"] = 4;

    auto mna = makeZeroMNA(5);
    std::vector<double> rhs(5, 0.0);

    ev->stamp(mna, rhs, indexMap);

    int vplus = indexMap["A"];
    int vminus = indexMap["B"];
    int i = indexMap["E1"];
    int vxplus = indexMap["X"];
    int vxminus = indexMap["Y"];
    double gain = 2.0;

    EXPECT_DOUBLE_EQ(mna[vplus][i], 1.0);
    EXPECT_DOUBLE_EQ(mna[vminus][i], -1.0);
    EXPECT_DOUBLE_EQ(mna[i][vplus], 1.0);
    EXPECT_DOUBLE_EQ(mna[i][vminus], -1.0);
    EXPECT_DOUBLE_EQ(mna[i][vxplus], -gain);
    EXPECT_DOUBLE_EQ(mna[i][vxminus], gain);
}

TEST(DependentVoltageStamp, CCVS_NodeToGround_UsesControllingBranch)
{
    auto ctrl = std::make_shared<CurrentSource>("I_CTRL", "C", "D", 1.0);
    auto ev =
        std::make_shared<TestDependentVoltageSource>("E2", "N1", "0", 3.0);
    ev->setControllingVariableEnum(ControlVariable::i);
    ev->setControllingElementPtr(ctrl);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["I_CTRL"] = 1;
    indexMap["E2"] = 2;

    auto mna = makeZeroMNA(3);
    std::vector<double> rhs(3, 0.0);

    ev->stamp(mna, rhs, indexMap);

    int vplus = indexMap["N1"];
    int is = indexMap["E2"];
    int ix = indexMap["I_CTRL"];
    double mu = 3.0;

    EXPECT_DOUBLE_EQ(mna[vplus][is], 1.0);
    EXPECT_DOUBLE_EQ(mna[is][vplus], 1.0);
    EXPECT_DOUBLE_EQ(mna[is][ix], -mu);
}
