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
#include "DependentCurrentSource.hpp"
#include "Parser.hpp"
#include "VoltageSource.hpp"

class TestDependentCurrentSource : public DependentCurrentSource
{
   public:
    using DependentCurrentSource::DependentCurrentSource;

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

TEST(DependentCurrentParse, ValidParseBasic)
{
    Parser parser;
    std::vector<std::string> tokens = {"IC1", "1", "0", "2", "V", "VCTRL"};
    auto el = DependentCurrentSource::parse(parser, tokens, 1);
    ASSERT_NE(el, nullptr);
    EXPECT_EQ(el->getName(), "IC1");
    EXPECT_EQ(el->getNodeA(), "1");
    EXPECT_EQ(el->getNodeB(), "0");
}

TEST(DependentCurrentParse, InvalidTokenCount)
{
    Parser parser;
    std::vector<std::string> tokens = {"IC1", "1", "0", "2", "V"};
    testing::internal::CaptureStderr();
    auto el = DependentCurrentSource::parse(parser, tokens, 2);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Invalid dependent current source definition"),
              std::string::npos);
}

TEST(DependentCurrentParse, SameNodesRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"ICBAD", "N1", "N1", "1", "V", "X1"};
    testing::internal::CaptureStderr();
    auto el = DependentCurrentSource::parse(parser, tokens, 3);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Dependent current source nodes cannot be the same"),
              std::string::npos);
}

TEST(DependentCurrentParse, IllegalZeroValue)
{
    Parser parser;
    std::vector<std::string> tokens = {"ICZ", "N1", "N2", "0", "V", "R1"};
    testing::internal::CaptureStderr();
    auto el = DependentCurrentSource::parse(parser, tokens, 4);
    std::string err = testing::internal::GetCapturedStderr();
    EXPECT_EQ(el, nullptr);
    EXPECT_NE(err.find("Illegal argument for dependent current source value"),
              std::string::npos);
}

TEST(DependentCurrentParse, IllegalControllingVariable)
{
    Parser parser;
    std::vector<std::string> tokens = {"ICX", "A", "B", "1", "Z", "X1"};
    testing::internal::CaptureStderr();
    auto el = DependentCurrentSource::parse(parser, tokens, 5);
    std::string err = testing::internal::GetCapturedStderr();
    ASSERT_NE(el, nullptr);
    EXPECT_NE(err.find("Illegal controlling variable argument"),
              std::string::npos);
}

TEST(DependentCurrentParse, CascadingControlledSourceRejected)
{
    Parser parser;
    std::vector<std::string> tokens = {"IG", "1", "0",
                                       "5",  "I", "IC_SOMETHING"};
    testing::internal::CaptureStderr();
    auto el = DependentCurrentSource::parse(parser, tokens, 6);
    std::string err = testing::internal::GetCapturedStderr();
    ASSERT_NE(el, nullptr);
    EXPECT_NE(err.find("cannot be cascaded"), std::string::npos);
}

TEST(DependentCurrentStamp, VCCS_BetweenNodes_UsesControllingNodes)
{
    auto ctrl = std::make_shared<VoltageSource>("VCTRL", "X", "Y", 5.0);
    auto g = std::make_shared<TestDependentCurrentSource>("G1", "A", "B", 2.0);
    g->setControllingVariableEnum(ControlVariable::v);
    g->setControllingElementPtr(ctrl);

    std::map<std::string, int> indexMap;
    indexMap["A"] = 0;
    indexMap["B"] = 1;
    indexMap["X"] = 2;
    indexMap["Y"] = 3;

    auto mna = makeZeroMNA(4);
    std::vector<double> rhs(4, 0.0);

    g->stamp(mna, rhs, indexMap);

    int vplus = indexMap["A"];
    int vminus = indexMap["B"];
    int vxplus = indexMap["X"];
    int vxminus = indexMap["Y"];
    double gm = 2.0;

    EXPECT_DOUBLE_EQ(mna[vplus][vxplus], gm);
    EXPECT_DOUBLE_EQ(mna[vplus][vxminus], -gm);
    EXPECT_DOUBLE_EQ(mna[vminus][vxplus], -gm);
    EXPECT_DOUBLE_EQ(mna[vminus][vxminus], gm);
}

TEST(DependentCurrentStamp, CCCS_BetweenNodes_UsesControllingBranch)
{
    auto ctrl = std::make_shared<CurrentSource>("I_CTRL", "C", "D", 1.0);
    auto f =
        std::make_shared<TestDependentCurrentSource>("F1", "Na", "Nb", 0.5);
    f->setControllingVariableEnum(ControlVariable::i);
    f->setControllingElementPtr(ctrl);

    std::map<std::string, int> indexMap;
    indexMap["Na"] = 0;
    indexMap["Nb"] = 1;
    indexMap["I_CTRL"] = 2;

    auto mna = makeZeroMNA(3);
    std::vector<double> rhs(3, 0.0);

    f->stamp(mna, rhs, indexMap);

    int vplus = indexMap["Na"];
    int vminus = indexMap["Nb"];
    int iidx = indexMap["I_CTRL"];
    double gain = 0.5;

    EXPECT_DOUBLE_EQ(mna[vplus][iidx], gain);
    EXPECT_DOUBLE_EQ(mna[vminus][iidx], -gain);
}

TEST(DependentCurrentStamp, CCCS_NodeToGround)
{
    auto ctrl = std::make_shared<CurrentSource>("I2", "P", "Q", 1.0);

    auto f =
        std::make_shared<TestDependentCurrentSource>("F2", "N1", "0", 1.25);
    f->setControllingVariableEnum(ControlVariable::i);
    f->setControllingElementPtr(ctrl);

    std::map<std::string, int> indexMap;
    indexMap["N1"] = 0;
    indexMap["I2"] = 1;

    auto mna = makeZeroMNA(2);
    std::vector<double> rhs(2, 0.0);

    f->stamp(mna, rhs, indexMap);

    EXPECT_DOUBLE_EQ(mna[0][indexMap["I2"]], 1.25);
}
