#pragma once

#include <memory>
#include <string>

#include "CircuitElement.hpp"
#include "NonlinearModel.hpp"

/**
 * @brief Nonlinear inductor element using a flux-based constitutive model.
 *
 * Implements TR companion linearization via `computeCompanionIter` which is
 * intended to be called inside Newton iterations. The class provides the
 * transient stamp and state update methods following the project's Group-2
 * branch-current conventions.
 */
class NonlinearInductor : public CircuitElement
{
   public:
    NonlinearInductor(const std::string &name, const std::string &nodeA,
                      const std::string &nodeB, double value,
                      std::shared_ptr<NonlinearModel> model = nullptr)
        : CircuitElement(name, nodeA, nodeB, value), model_(std::move(model))
    {
        if (!model_) {
            // default linear flux model phi = L * i
            model_ = makePolynomialFluxModel({0.0, value});
        }
        group = Group::G2;  // inductors are group-2 in this codebase
    }

    bool isNonlinear() const override { return true; }

    // DC stamp: in steady-state an inductor is short; implement in CPP so
    // the DC operating point contains branch variables for initialisation.
    void stamp(std::vector<std::vector<double>> &mna, std::vector<double> &rhs,
               std::map<std::string, int> &indexMap) override;

    void computeCompanion(double h) override;

    void computeCompanionIter(
        double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) override;

    void stampTransient(std::vector<std::vector<double>> &mna,
                        std::vector<double> &rhs,
                        std::map<std::string, int> &indexMap) override;

    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const std::map<std::string, int> &indexMap) override;

    // Diagnostics dump for solver debugging
    void dumpDiagnostics(
        std::ostream &os, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) const override;

    // Checkpoint/restore
    std::vector<double> snapshotState() const override;
    void restoreState(const std::vector<double> &data) override;

   private:
    std::shared_ptr<NonlinearModel> model_;

    // Stored states
    double i_prev_ = 0.0;    // branch current at previous timestep
    double u_prev_ = 0.0;    // voltage across inductor at previous timestep
    double phi_prev_ = 0.0;  // flux at previous timestep

    // Companion parameters for current Newton iteration
    double Geq_ = 0.0;  // algebraic helper (h / (2 * dphidi))
    double Ieq_ = 0.0;  // companion constant used in branch equation
};
