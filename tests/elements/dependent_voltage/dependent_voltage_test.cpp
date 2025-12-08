/*
 * SNU_Spice/tests/elements/dependent_voltage/dependent_voltage_test.cpp
 *
 * Unit tests for DependentVoltageSource::parse(...) and basic stamping checks.
 *
 * These tests verify:
 *  - parsing validation (token count, same nodes, illegal values, illegal
 *    controlling variable, cascading rejection)
 *  - basic stamping patterns for VCVS (voltage-controlled voltage source)
 *    and CCVS (current-controlled voltage source). To safely set the
 *    controlling element pointer (protected member in the base class),
 *    a small test-derived class exposes setters for use by the tests.
 *
 * The tests avoid inspecting private companion/internal members and instead
 * validate observable MNA matrix entries produced by `stamp(...)`.
 */

#include <gtest/gtest.h>

#include "DependentVoltageSource.hpp"
#include "VoltageSource.hpp"
#include "CurrentSource.hpp"
#include "Parser.hpp"
#include "CircuitElement.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

/*
 * Test helper: subclass DependentVoltageSource to expose protected members for
 * testing. This allows tests to inject a controlling element pointer and to
 * explicitly set the controlling variable enum.
 */
class TestDependentVoltageSource : public DependentVoltageSource
{
  public:
    using DependentVoltageSource::DependentVoltageSource;

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

TEST(DependentVoltageParse, ValidParseBasic)
{
  Parser parser;
  // tokens: name nodeA nodeB value controlling_variable controlling_element
  std::vector<std::string> tokens = {"VC1", "1", "0", "2", "V", "VCTRL"};
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/1);
  ASSERT_NE(el, nullptr);
  // Basic identity checks
  EXPECT_EQ(el->getName(), "VC1");
  EXPECT_EQ(el->getNodeA(), "1");
  EXPECT_EQ(el->getNodeB(), "0");
}

TEST(DependentVoltageParse, InvalidTokenCount)
{
  Parser parser;
  std::vector<std::string> tokens = {"VC1", "1", "0", "2", "V"}; // missing controlling element
  testing::internal::CaptureStderr();
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/2);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Invalid dependent voltage source definition"), std::string::npos);
}

TEST(DependentVoltageParse, SameNodesRejected)
{
  Parser parser;
  std::vector<std::string> tokens = {"Vbad", "N1", "N1", "1", "V", "X1"};
  testing::internal::CaptureStderr();
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/3);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Dependent voltage source nodes cannot be the same"), std::string::npos);
}

TEST(DependentVoltageParse, IllegalZeroValue)
{
  Parser parser;
  std::vector<std::string> tokens = {"VZ", "N1", "N2", "0", "V", "V1"};
  testing::internal::CaptureStderr();
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/4);
  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_EQ(el, nullptr);
  EXPECT_NE(err.find("Illegal argument for dependent voltage source value"), std::string::npos);
}

TEST(DependentVoltageParse, IllegalControllingVariable)
{
  Parser parser;
  std::vector<std::string> tokens = {"Vx", "A", "B", "1", "Q", "X1"};
  testing::internal::CaptureStderr();
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/5);
  std::string err = testing::internal::GetCapturedStderr();
  // parse still returns an element but prints an illegal controlling variable diagnostic
  ASSERT_NE(el, nullptr);
  EXPECT_NE(err.find("Illegal controlling variable argument"), std::string::npos);
}

TEST(DependentVoltageParse, CascadingControlledSourceRejected)
{
  Parser parser;
  // controlling element name begins with "VC" -> cascade not allowed
  std::vector<std::string> tokens = {"VE", "1", "0", "5", "V", "VC_SOMETHING"};
  testing::internal::CaptureStderr();
  auto el = DependentVoltageSource::parse(parser, tokens, /*lineNumber=*/6);
  std::string err = testing::internal::GetCapturedStderr();
  // parse returns an element but emits a cascade-warning/error message
  ASSERT_NE(el, nullptr);
  EXPECT_NE(err.find("cannot be cascaded"), std::string::npos);
}

// ---------------------- Stamping tests ----------------------

// VCVS (Voltage-Controlled Voltage Source) stamping when controlling element exists
TEST(DependentVoltageStamp, VCVS_BetweenNodes_UsesControllingNodes)
{
  // Create a controlling voltage source (the driver)
  auto ctrl = std::make_shared<VoltageSource>("VCTRL", "X", "Y", 5.0);

  // Create a test dependent voltage source (E-type) with gain = 2.0
  auto ev = std::make_shared<TestDependentVoltageSource>("E1", "A", "B", 2.0);
  ev->setControllingVariableEnum(ControlVariable::v);
  ev->setControllingElementPtr(ctrl);

  // index map: assign indices for nodes and branch unknown for E1 and node references of controller
  // Layout: A=0, B=1, X=2, Y=3, E1(branch)=4
  std::map<std::string,int> indexMap;
  indexMap["A"] = 0;
  indexMap["B"] = 1;
  indexMap["X"] = 2; // ctrl nodeA
  indexMap["Y"] = 3; // ctrl nodeB
  indexMap["E1"] = 4;

  auto mna = makeZeroMNA(5);
  std::vector<double> rhs(5, 0.0);

  // Stamp the VCVS. According to implementation, when both E1 nodes and ctrl nodes are non-ground:
  // mna[A][i] += 1.0
  // mna[B][i] += -1.0
  // mna[i][A] += 1.0
  // mna[i][B] += -1.0
  // mna[i][vxplus] += -value
  // mna[i][vxminus] += value
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

  // Note sign conventions in the implementation: for the case where both ctrl nodes
  // are not ground, the code adds mna[i][vxplus] += -value and mna[i][vxminus] += value
  EXPECT_DOUBLE_EQ(mna[i][vxplus], -gain);
  EXPECT_DOUBLE_EQ(mna[i][vxminus], gain);
}

// CCVS (Current-Controlled Voltage Source) stamping when controlling element is present
TEST(DependentVoltageStamp, CCVS_NodeToGround_UsesControllingBranch)
{
  // Create a controlling current-like element (it must have a name in indexMap,
  // representing its branch current index). We'll use a CurrentSource instance
  // as the controlling element (tests inject branch index manually).
  auto ctrl = std::make_shared<CurrentSource>("I_CTRL", "C", "D", 1.0);

  // Dependent voltage source: name 'E2', between node 'N1' and ground
  auto ev = std::make_shared<TestDependentVoltageSource>("E2", "N1", "0", 3.0);
  ev->setControllingVariableEnum(ControlVariable::i);
  ev->setControllingElementPtr(ctrl);

  // indexMap layout: N1=0, I_CTRL(branch)=1, E2(branch)=2
  std::map<std::string,int> indexMap;
  indexMap["N1"] = 0;
  indexMap["I_CTRL"] = 1; // branch index of controlling element
  indexMap["E2"] = 2;     // branch index of dependent source

  auto mna = makeZeroMNA(3);
  std::vector<double> rhs(3, 0.0);

  // For CCVS with nodeB == 0 (target grounded), implementation should:
  // mna[vplus][is] += 1.0;
  // mna[is][vplus] += 1.0;
  // mna[is][ix] += -value;
  ev->stamp(mna, rhs, indexMap);

  int vplus = indexMap["N1"];
  int is = indexMap["E2"];
  int ix = indexMap["I_CTRL"];
  double mu = 3.0;

  EXPECT_DOUBLE_EQ(mna[vplus][is], 1.0);
  EXPECT_DOUBLE_EQ(mna[is][vplus], 1.0);
  EXPECT_DOUBLE_EQ(mna[is][ix], -mu);
}

// End of file - gtest_main supplies test runner.