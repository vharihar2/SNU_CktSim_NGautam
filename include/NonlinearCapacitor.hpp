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
#pragma once

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "CircuitElement.hpp"
#include "NonlinearModel.hpp"

/**
 * @file NonlinearCapacitor.hpp
 * @brief Nonlinear capacitor element using a charge-based constitutive model.
 *
 * This header declares `NonlinearCapacitor`, an element that models a capacitor
 * whose charge-voltage relation q = q(u) may be nonlinear. The element
 * implements a trapezoidal-rule (TR) companion linearization suitable for
 * Newton-based transient solves:
 *
 *  - A per-timestep companion may be computed via `computeCompanion(h)` (a
 *    simple fallback that linearizes around the stored previous state).
 *  - During Newton iterations the more accurate linearization is produced by
 *    `computeCompanionIter(h, xk, indexMap)` which evaluates the model at the
 *    current Newton iterate and produces a linearized Norton equivalent
 *    (`Geq_`, `Ieq_`).
 *
 * The class also provides transient stamping (`stampTransient`), state
 * updates (`updateStateFromSolution`), diagnostics (`dumpDiagnostics`) and
 * checkpoint/restore helpers (`snapshotState` / `restoreState`).
 *
 * Notes:
 *  - For DC analysis the capacitor behaves as an open circuit (DC stamp is
 *    intentionally a no-op).
 *  - The `model_` is a `NonlinearModel` instance that provides `q(u)` and
 *    `dqdu(u)`; when no model is supplied a linear polynomial model with
 *    coefficient `C` (the element value) is used.
 */
class NonlinearCapacitor : public CircuitElement
{
   public:
    /**
     * @brief Construct a new NonlinearCapacitor.
     *
     * @param name Element identifier (e.g., \"C1\").
     * @param nodeA Positive node name.
     * @param nodeB Negative node name.
     * @param value Nominal capacitance value (used by default linear model).
     * @param model Optional nonlinear charge model. If null, a linear charge
     *              model q = C * u is created automatically.
     */
    NonlinearCapacitor(const std::string &name, const std::string &nodeA,
                       const std::string &nodeB, double value,
                       std::shared_ptr<NonlinearModel> model = nullptr)
        : CircuitElement(name, nodeA, nodeB, value), model_(std::move(model))
    {
        // Default to a linear charge model q = C*u when no model supplied.
        if (!model_) {
            // polynomial coeffs: a0=0, a1=C
            model_ = makePolynomialChargeModel({0.0, value});
        }
        // Nonlinear capacitors participate in transient stamping as Group 2
        // elements (branch-based formulation).
        group = Group::G2;
    }

    /** @brief Nonlinear devices return true. */
    bool isNonlinear() const override { return true; }

    /**
     * @brief DC stamping (open-circuit).
     *
     * For DC operating-point analysis an ideal capacitor is an open circuit;
     * therefore the DC stamp is a no-op.
     */
    void stamp(std::vector<std::vector<double>> & /*mna*/,
               std::vector<double> & /*rhs*/,
               std::map<std::string, int> & /*indexMap*/) override
    {
        // no-op for DC (open circuit)
    }

    /**
     * @brief Compute a simple per-timestep companion linearization.
     *
     * This fallback linearization evaluates the model at the stored previous
     * state (u_prev_) and sets `Geq_` and `Ieq_` accordingly:
     *   Geq_ = (2/h) * dq/du |_{u_prev}
     *   Ieq_ = Geq_ * u_prev - i_prev
     *
     * Implementations should guard against non-positive `h`.
     */
    void computeCompanion(double h) override;

    /**
     * @brief Per-Newton-iteration companion computation (TR linearization).
     *
     * Evaluates the nonlinear model at the current Newton iterate `xk` and
     * computes the linearized companion parameters (`Geq_`, `Ieq_`) used for
     * stamping the element's Jacobian/contribution during iterative solves.
     *
     * The method must read node voltages from `xk` using `indexMap` (ground
     * node
     * \"0\" is handled specially) and clamp/validate model derivatives to avoid
     * ill-conditioned companion entries.
     */
    void computeCompanionIter(
        double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) override;

    /**
     * @brief Stamp the transient companion (Norton) into `mna` and `rhs`.
     *
     * Preconditions: `computeCompanion` or `computeCompanionIter` must have
     * populated `Geq_` and `Ieq_` for the current timestep/iterate.
     *
     * The stamp handles ground cases (node name \"0\") and follows the
     * repository-wide sign convention: companion current `Ieq_` is injected as
     * -Ieq at nodeA and +Ieq at nodeB.
     */
    void stampTransient(std::vector<std::vector<double>> &mna,
                        std::vector<double> &rhs,
                        std::map<std::string, int> &indexMap) override;

    /**
     * @brief Update internal state from the solver solution vector.
     *
     * Reads node voltages from `x` (using `indexMap`), computes the capacitor
     * voltage at the new timestep and updates `u_prev_`, `q_prev_` and
     * `i_prev_` using the companion relation.
     */
    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const std::map<std::string, int> &indexMap) override;

    /**
     * @brief Emit a compact diagnostic snapshot for the given iterate.
     *
     * This helper prints the element name, nodes, current iterate voltage,
     * charge and derivative information plus the active companion parameters.
     */
    void dumpDiagnostics(
        std::ostream &os, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) const override;

    /** @brief Return a compact snapshot of transient state (u_prev_, i_prev_,
     * q_prev_). */
    std::vector<double> snapshotState() const override;

    /** @brief Restore transient state produced by snapshotState(). */
    void restoreState(const std::vector<double> &data) override;

   private:
    /** @brief Nonlinear charge model q(u) and its derivative dq/du. */
    std::shared_ptr<NonlinearModel> model_;

    /* Stored states at time n (previous step) */
    double u_prev_ = 0.0; /**< Voltage across capacitor at previous step (V) */
    double i_prev_ = 0.0; /**< Branch current at previous step (A) */
    double q_prev_ = 0.0; /**< Stored charge at previous step (C) */

    /* Companion parameters for current Newton iteration / timestep */
    double Geq_ = 0.0; /**< Equivalent conductance for companion (S) */
    double Ieq_ = 0.0; /**< Equivalent Norton current source (A) */
};
