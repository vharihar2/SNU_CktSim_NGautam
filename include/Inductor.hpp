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

#include <string>

#include "CircuitElement.hpp"

// Forward declaration for parser used by the parse factory.
class Parser;

/**
 * @file Inductor.hpp
 * @brief Declaration of the linear Inductor element.
 *
 * This header declares the `Inductor` class which models an ideal linear
 * inductor for both DC and transient (time-domain) simulations. The class
 * derives from `CircuitElement` and implements interfaces used by the solver
 * for DC stamping, transient companion computation (trapezoidal-rule), and
 * state updates after a transient solution.
 *
 * Transient companion (trapezoidal rule) conventions used in the simulator:
 *   Geq = h / (2 * L)
 *   Ieq = i_n + Geq * u_n
 *
 * where h is the timestep, L is the inductance (value), and i_n / u_n are the
 * element's previous-step current and voltage respectively.
 */

/**
 * @class Inductor
 * @brief Ideal linear inductor element.
 *
 * The `Inductor` represents an ideal inductance with a scalar inductance value
 * (in Henries). For DC operating-point analysis an ideal inductor is typically
 * represented by its branch-current unknown (Group G2). For transient analysis
 * the class supports formation of a Trapezoidal-Rule companion (Norton/series)
 * to integrate the element in the time domain.
 */
class Inductor : public CircuitElement
{
   private:
    /** @brief Inductor current at the previous timestep (i_n). Units: Amperes.
     */
    double i_prev = 0.0;

    /** @brief Inductor voltage at the previous timestep (u_n). Units: Volts. */
    double u_prev = 0.0;

    /**
     * @brief Equivalent conductance (or related coefficient) for the per-step
     * companion model.
     *
     * For the trapezoidal companion used in this codebase:
     *   Geq = h / (2 * L)
     * Units: Siemens (or the appropriate reciprocal unit given the companion
     * formulation).
     */
    double Geq = 0.0;

    /**
     * @brief Equivalent companion source term for the current-step model.
     *
     * For trapezoidal companion: Ieq = i_n + Geq * u_n (see file-level docs).
     * Units: Amperes.
     */
    double Ieq = 0.0;

   public:
    /**
     * @brief Construct a new Inductor element.
     *
     * @param name Element identifier (e.g., "L1").
     * @param nodeA Positive/first node name (netlist token 1).
     * @param nodeB Negative/second node name (netlist token 2).
     * @param value Inductance value in Henries (netlist token 3). Must be
     * non-zero.
     */
    Inductor(const std::string& name, const std::string& nodeA,
             const std::string& nodeB, double value)
        : CircuitElement(name, nodeA, nodeB, value)
    {
    }

    /**
     * @brief Stamp the inductor for DC/MNA assembly.
     *
     * For Group-2 inductor formulations this typically adds the coupling terms
     * between node voltages and the branch-current unknown and sets up the
     * branch-current row/column. Implementations should follow the project's
     * MNA conventions (see implementation file).
     *
     * @param mna Modified Nodal Analysis matrix (mutated in-place).
     * @param rhs Right-hand side vector (mutated in-place).
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    virtual void stamp(std::vector<std::vector<double>>& mna,
                       std::vector<double>& rhs,
                       std::map<std::string, int>& indexMap) override;

    /**
     * @brief Parse an inductor from a tokenized netlist line.
     *
     * Expected token format:
     *   Lname nodeA nodeB value
     *
     * On success returns a `std::shared_ptr<CircuitElement>` pointing to the
     * created `Inductor`. On failure returns `nullptr` and emits diagnostics
     * via the parser/IO layer.
     *
     * @param parser Parser helper used for validation and numeric parsing.
     * @param tokens Tokenized netlist line.
     * @param lineNumber Line number in the netlist (for diagnostics).
     * @return std::shared_ptr<CircuitElement> Created Inductor or nullptr on
     * error.
     */
    static std::shared_ptr<CircuitElement> parse(
        Parser& parser, const std::vector<std::string>& tokens, int lineNumber);

    /**
     * @brief Compute trapezoidal-rule companion parameters for timestep `h`.
     *
     * Computes and stores `Geq` and `Ieq` for use by `stampTransient`.
     * Convention used here:
     *   Geq = h / (2 * L)
     *   Ieq = i_n + Geq * u_n
     *
     * Implementations should guard against non-positive `h`.
     *
     * @param h Time-step size in seconds.
     */
    void computeCompanion(double h) override;

    /**
     * @brief Stamp the transient companion contribution into `mna` and `rhs`.
     *
     * Precondition: `computeCompanion(h)` has been called for the current
     * timestep so that `Geq` and `Ieq` are valid. This method stamps either the
     * branch-series formulation (if a branch-current unknown exists) or a
     * Norton-equivalent between nodes as a fallback.
     *
     * @param mna Modified Nodal Analysis matrix (mutated in-place).
     * @param rhs Right-hand side vector (mutated in-place).
     * @param indexMap Map from node/element names to indices in `mna`/`rhs`.
     */
    void stampTransient(std::vector<std::vector<double>>& mna,
                        std::vector<double>& rhs,
                        std::map<std::string, int>& indexMap) override;

    /**
     * @brief Update element internal state (i_prev, u_prev) from the solver
     * result.
     *
     * Reads node voltages (and branch-current unknown if present) from the
     * solver's solution vector `x` using `indexMap` and updates `i_prev` and
     * `u_prev` for the next timestep.
     *
     * @param x Solution vector produced by the solver for the current timestep.
     * @param indexMap Map from node/element names to indices in `x`.
     */
    void updateStateFromSolution(
        const Eigen::Ref<const Eigen::VectorXd>& x,
        const std::map<std::string, int>& indexMap) override;
};
