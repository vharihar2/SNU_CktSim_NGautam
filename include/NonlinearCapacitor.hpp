#pragma once

#include <memory>
#include <string>

#include "CircuitElement.hpp"
#include "NonlinearModel.hpp"

/**
 * @brief Nonlinear capacitor element using a charge-based constitutive model.
 *
 * Implements TR companion linearization via `computeCompanionIter` which is
 * intended to be called inside Newton iterations. The class also provides the
 * transient stamp and state update methods.
 */
class NonlinearCapacitor : public CircuitElement
{
   public:
    NonlinearCapacitor(const std::string &name, const std::string &nodeA,
                       const std::string &nodeB, double value,
                       std::shared_ptr<NonlinearModel> model = nullptr)
        : CircuitElement(name, nodeA, nodeB, value), model_(std::move(model))
    {
        // default to a linear charge model q = C*u when no model supplied
        if (!model_) {
            // polynomial coeffs: a0=0, a1=C
            model_ = makePolynomialChargeModel({0.0, value});
        }
        group = Group::G2;  // capacitors handled as group-2 transient elements
    }

    bool isNonlinear() const override { return true; }

    // DC stamp: capacitor is open in DC
    void stamp(std::vector<std::vector<double>> & /*mna*/,
               std::vector<double> & /*rhs*/,
               std::map<std::string, int> & /*indexMap*/) override
    {
        // no-op for DC (open circuit)
    }

    void computeCompanion(double h) override;

    // TR per-Newton companion computation
    void computeCompanionIter(
        double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) override;

    void stampTransient(std::vector<std::vector<double>> &mna,
                        std::vector<double> &rhs,
                        std::map<std::string, int> &indexMap) override;

    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const std::map<std::string, int> &indexMap) override;

   private:
    std::shared_ptr<NonlinearModel> model_;

    // Stored states at time n
    double u_prev_ = 0.0;  // voltage across capacitor at previous step
    double i_prev_ = 0.0;  // branch current at previous step
    double q_prev_ = 0.0;  // stored charge at previous step

    // Companion parameters for current Newton iteration
    double Geq_ = 0.0;
    double Ieq_ = 0.0;
};
