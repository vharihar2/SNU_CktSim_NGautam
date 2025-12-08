/*
 * SNU_Spice/tests/elements/dependent_current/dependent_current_test.cpp
 *
 * Unit tests for DependentCurrentSource::parse(...) and basic stamping checks.
 *
 * These tests mirror the style of the dependent voltage tests:
 *  - parsing validation (token count, same nodes, illegal values, illegal
 *    controlling variable, cascading rejection)
 *  - stamping behavior for VCCS (voltage-controlled current source) and
 *    CCCS (current-controlled current source) using a small test helper that
 *    can inject a controlling element pointer.
 *
 * The tests validate observable MNA matrix entries and diagnostics printed to
 * stderr; they do not access private members directly.
 */

#include <gtest/gtest.h>

#include "DependentCurrentSource.hpp"
#include "CurrentSource.hpp"
#include "VoltageSource.hpp"
#include "Parser.hpp"
#include "CircuitElement.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

/*
 * Test helper: subclass DependentCurrentSource to expose protected members for
 * testing. This allows tests to inject a controlling element pointer and to
 * explicitly set the controlling variable enum.
 */
class TestDependentCurrentSource : public DependentCurrentSource
{
  public:
    using DependentCurrentSource::DependentCurrentSource;

    void setControllingElementPtr(std::shared_ptr<CircuitElement> el)
    {
        this->controlling_element = std::move(el);
    }

    void setControllingVariableEnum(ControlVariable cv) { this->controlling_variable = cv; }
};

// small helper to build an NxN zero matrix
static std::vector<std::vector<double>> makeZeroMNA(size_t n)
{
  return std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
}

// ---------------------- Parsing tests ----------------------

TEST(DependentCurrentParse, ValidParseBasic)
{
  Parser parser;
  // tokens: name nodeA nodeB value controlling_variable controlling_element
  std::vector<std::string> tokens = {"IC1", "1", "0", "2", "V", "VCTRL"};
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  // Basic checks
  EXPECT_EQ(el->getName(), "IC1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
}

TEST(DependentCurrentParse, InvalidTokenCount)
{
  Parser parser;
  std::vector<std::string> tokens = {"IC1", "1", "0", "2", "V"}; // missing controlling element
  testing::internal::CaptureStderr();
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid dependent current source definition"), std::string::npos);
}

TEST(DependentCurrentParse, SameNodesRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"ICBAD", "N1", "N1", "1", "V", "X1"};
  testing::internal::CaptureStderr();
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Dependent current source nodes cannot be the same"), std::string::npos);
}

TEST(DependentCurrentParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"ICZ", "N1", "N2", "0", "V", "R1"};
  testing::internal::CaptureStderr();
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/4);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for dependent current source value"), std::string::npos);
}

TEST(DependentCurrentParse, IllegalControllingVariable)
{
  Parser parser;
  std::vector<std::string> tokens = {"ICX", "A", "B", "1", "Z", "X1"};
  testing::internal::CaptureStderr();
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();
  // parse still returns an element but prints an illegal controlling variable diagnostic
  ASSERT_NE(el, nullptr);
  EXPECT_NE(err.find("Illegal controlling variable argument"), std::string::npos);
}

TEST(DependentCurrentParse, CascadingControlledSourceRejected)
{
  Parser parser;
  // controlling element name begins with "IC" -> cascade not allowed
  std::vector<std::string> tokens = {"IG", "1", "0", "5", "I", "IC_SOMETHING"};
  testing::internal::CaptureStderr();
  auto el = DependentCurrentSource::parse(parser, tokens, /*lineNumber=*/6);
  std::string err = testing::internal::GetCapturedStderr();
  // parse returns an element but emits a cascade-warning/error message
  ASSERT_NE(el, nullptr);
  EXPECT_NE(err.find("cannot be cascaded"), std::string::npos);
}

// ---------------------- Stamping tests ----------------------

// VCCS (Voltage-Controlled Current Source) stamping when controlling element exists
TEST(DependentCurrentStamp, VCCS_BetweenNodes_UsesControllingNodes)
{
  // Create a controlling voltage source (the driver)
  auto ctrl = std::make_shared<VoltageSource>("VCTRL", "X", "Y", 5.0);

  // Create a test dependent current source with transconductance = 2.0
  auto g = std::make_shared<TestDependentCurrentSource>("G1", "A", "B", 2.0);
  g->setControllingVariableEnum(ControlVariable::v);
  g->setControllingElementPtr(ctrl);

  // index map: A=0, B=1, X=2, Y=3
  std::map<std::string,int> indexMap;
  indexMap["A"] = 0;
  indexMap["B"] = 1;
  indexMap["X"] = 2; // ctrl nodeA
  indexMap["Y"] = 3; // ctrl nodeB

  auto mna = makeZeroMNA(4);
  std::vector<double> rhs(4, 0.0);

  // For the case where both control nodes are not ground, and G is between two nodes:
  // Implementation should add:
  // mna[vplus][vxplus] += value;
  // mna[vplus][vxminus] += -value;
  // mna[vminus][vxplus] += -value;
  // mna[vminus][vxminus] += value;
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

// CCCS (Current-Controlled Current Source) stamping when controlling element is present
TEST(DependentCurrentStamp, CCCS_BetweenNodes_UsesControllingBranch)
{
  // Create a controlling current-like element (it must have a name in indexMap,
  // representing its branch current index). Use CurrentSource instance as the controller.
  auto ctrl = std::make_shared<CurrentSource>("I_CTRL", "C", "D", 1.0);

  // Dependent current source: name 'F1', between nodes 'Na' and 'Nb', transresistance value = 0.5
  // (In our implementation 'value' is the factor multiplying controlling current.)
  auto f = std::make_shared<TestDependentCurrentSource>("F1", "Na", "Nb", 0.5);
  f->setControllingVariableEnum(ControlVariable::i);
  f->setControllingElementPtr(ctrl);

  // indexMap layout: Na=0, Nb=1, I_CTRL(branch)=2
  std::map<std::string,int> indexMap;
  indexMap["Na"] = 0;
  indexMap["Nb"] = 1;
  indexMap["I_CTRL"] = 2;

  auto mna = makeZeroMNA(3);
  std::vector<double> rhs(3, 0.0);

  // For CCCS between two non-ground nodes, implementation does:
  // mna[vplus][i] += value;
  // mna[vminus][i] += -value;
  f->stamp(mna, rhs, indexMap);

  int vplus = indexMap["Na"];
  int vminus = indexMap["Nb"];
  int iidx = indexMap["I_CTRL"];
  double gain = 0.5;

  EXPECT_DOUBLE_EQ(mna[vplus][iidx], gain);
  EXPECT_DOUBLE_EQ(mna[vminus][iidx], -gain);
}

// CCCS when one node is ground: nodeA != 0, nodeB == 0
TEST(DependentCurrentStamp, CCCS_NodeToGround)
{
  auto ctrl = std::make_shared<CurrentSource>("I2", "P", "Q", 1.0);

  auto f = std::make_shared<TestDependentCurrentSource>("F2", "N1", "0", 1.25);
  f->setControllingVariableEnum(ControlVariable::i);
  f->setControllingElementPtr(ctrl);

  std::map<std::string,int> indexMap;
  indexMap["N1"] = 0;
  indexMap["I2"] = 1;

  auto mna = makeZeroMNA(2);
  std::vector<double> rhs(2, 0.0);

  // For nodeB == 0 (target grounded) the implementation does:
  // mna[vplus][i] += value;
  f->stamp(mna, rhs, indexMap);

  EXPECT_DOUBLE_EQ(mna[0][indexMap["I2"]], 1.25);
}

// End of file - gtest_main supplies test runner.