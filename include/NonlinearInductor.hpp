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
 * @file NonlinearInductor.hpp
 * @brief Declaration of the nonlinear inductor element (flux-based).
 *
 * This header declares `NonlinearInductor`, an element modeling an inductor
 * whose flux-current relation phi = phi(i) may be nonlinear. The element
 * supports trapezoidal-rule (TR) companion linearization for transient
 * Newton solves and follows the project's Group-2 branch-current stamping
 * conventions.
 *
 * Key behaviors:
 *  - DC stamp: creates the branch-current unknown and node-branch coupling
 *    entries so the DC operating-point solution provides initial `i_prev_`
 *    and `u_prev_` for the nonlinear model.
 *  - computeCompanion / computeCompanionIter: compute per-step or per-Newton
 *    iterate companion parameters (Geq_, Ieq_) used by `stampTransient`.
 *  - stampTransient: inserts series/branch or Norton-equivalent companion
 *    contributions into the MNA matrix and RHS.
 *  - updateStateFromSolution / snapshotState / restoreState: manage transient
 *    state across timesteps and retries.
 *
 * The `model_` member is a `NonlinearModel` providing `phi(i)` and `dphi/di`.
 * When no model is supplied, a linear flux model phi = L * i is created
 * automatically using the element `value`.
 */
class NonlinearInductor : public CircuitElement
{
   public:
    /**
     * @brief Construct a new NonlinearInductor.
     *
     * @param name Element identifier (e.g., "L1").
     * @param nodeA Positive node name (element terminal A).
     * @param nodeB Negative node name (element terminal B).
     * @param value Nominal inductance (used when creating default linear
     * model).
     * @param model Optional nonlinear flux model; if null a linear model is
     * created.
     */
    NonlinearInductor(const std::string &name, const std::string &nodeA,
                      const std::string &nodeB, double value,
                      std::shared_ptr<NonlinearModel> model = nullptr)
        : CircuitElement(name, nodeA, nodeB, value), model_(std::move(model))
    {
        if (!model_) {
            // default linear flux model phi = L * i (polynomial: a0=0, a1=L)
            model_ = makePolynomialFluxModel({0.0, value});
        }
        // Inductors participate as Group-2 elements (branch-current
        // formulation)
        group = Group::G2;
    }

    /** @brief Nonlinear device indicator (returns true). */
    bool isNonlinear() const override { return true; }

    /**
     * @brief Stamp the DC (operating-point) contribution.
     *
     * For nonlinear inductors the DC stamp creates the branch-current unknown
     * and coupling rows/columns so that the DC solve yields initial branch and
     * node voltages/currents for transient initialization.
     *
     * @param mna MNA matrix to modify.
     * @param rhs RHS vector to modify.
     * @param indexMap Map from node/element names to matrix indices.
     */
    void stamp(std::vector<std::vector<double>> &mna, std::vector<double> &rhs,
               std::map<std::string, int> &indexMap) override;

    /**
     * @brief Compute a per-step companion (fallback linearization).
     *
     * Default simple linearization uses stored state (phi_prev_, i_prev_)
     * to compute Geq_ and Ieq_ for the current timestep h.
     *
     * @param h Time-step size in seconds.
     */
    void computeCompanion(double h) override;

    /**
     * @brief Per-Newton-iterate companion computation for TR nonlinear solves.
     *
     * Called during Newton iterations to linearize the nonlinear flux-current
     * relation around the current iterate. Reads branch-current and node
     * voltages from `xk` using `indexMap` when available and updates Geq_/Ieq_
     * with safety clamps to avoid ill-conditioned entries.
     *
     * @param h Time-step size [s].
     * @param xk Current Newton iterate vector.
     * @param indexMap Map from node/element names to indices in `xk`.
     */
    void computeCompanionIter(
        double h, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) override;

    /**
     * @brief Stamp the transient companion (series branch or Norton fallback).
     *
     * Preconditions: `computeCompanion` or `computeCompanionIter` must have
     * populated `Geq_` and `Ieq_` for the current timestep/iterate.
     *
     * @param mna MNA matrix to modify.
     * @param rhs RHS vector to modify.
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    void stampTransient(std::vector<std::vector<double>> &mna,
                        std::vector<double> &rhs,
                        std::map<std::string, int> &indexMap) override;

    /**
     * @brief Update stored transient state from the solver solution.
     *
     * Reads node voltages and branch-current unknown (if present) from `x`
     * using `indexMap` and updates `i_prev_`, `u_prev_` and `phi_prev_`.
     *
     * @param x Solution vector produced by the solver.
     * @param indexMap Map from names to indices in `x`.
     */
    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd> &x,
        const std::map<std::string, int> &indexMap) override;

    /**
     * @brief Emit compact diagnostics for the current iterate.
     *
     * Useful when debugging nonlinear TR solves; prints element name, nodes,
     * branch current, flux and companion parameters.
     */
    void dumpDiagnostics(
        std::ostream &os, const Eigen::Ref<const Eigen::VectorXd> &xk,
        const std::map<std::string, int> &indexMap) const override;

    /** @brief Return a compact snapshot of transient state for checkpointing.
     */
    std::vector<double> snapshotState() const override;
    /** @brief Restore transient state previously returned by snapshotState().
     */
    void restoreState(const std::vector<double> &data) override;

   private:
    /** @brief Nonlinear flux model phi(i) and its derivative dphi/di. */
    std::shared_ptr<NonlinearModel> model_;

    /* Stored states at previous timestep (time n) */
    double i_prev_ = 0.0; /**< Branch current at previous timestep [A] */
    double u_prev_ =
        0.0; /**< Voltage across inductor at previous timestep [V] */
    double phi_prev_ =
        0.0; /**< Flux at previous timestep (model-dependent units) */

    /* Companion parameters for the current Newton iterate / timestep */
    double Geq_ = 0.0; /**< Algebraic helper (h / (2 * dphi/di)) */
    double Ieq_ = 0.0; /**< Companion constant used in branch equation */
};
