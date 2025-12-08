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

#include <stdexcept>
#include <string>

/*
 * SolverOptions.hpp
 *
 * Lightweight configuration container for runtime solver options.
 *
 * This header declares `SolverOptions`, a simple POD-style struct that carries
 * solver configuration knobs from the command-line driver (or tests) into the
 * solver implementation. The options control Newton damping, iteration limits,
 * diagnostic output and a few defensive runtime guards used by the transient
 * driver.
 *
 * Design goals:
 *  - Keep options small and trivial to copy (no heavy ownership semantics).
 *  - Provide a `validate()` method that checks for obviously invalid values
 *    and throws `std::invalid_argument` for fatal misconfiguration.
 *  - Document each option's purpose and typical use-cases so callers can tune
 *    behavior without reading the solver implementation.
 */
/**
 * @struct SolverOptions
 * @brief Runtime options controlling solver tolerances, diagnostics and safety.
 *
 * Fields in this struct are intentionally public to allow easy construction and
 * modification at the call-site (e.g., parsing CLI flags). Callers should
 * invoke `validate()` after setting options to ensure values are sensible.
 */
struct SolverOptions
{
    /**
     * @brief Damping factor applied to Newton updates.
     *
     * After computing a full Newton step, the solver forms x_{new} = x_old +
     * alpha * (delta) where `alpha` is `newtonAlpha`. Typical values:
     *  - 1.0 : no damping (full Newton step)
     *  - 0.5 : conservative half-step
     *
     * The transient driver will still perform adaptive backtracking if the
     * full/damped iterate leads to non-finite values or increases the residual.
     */
    double newtonAlpha = 1.0;

    /**
     * @brief Maximum number of Newton iterations per time-step.
     *
     * The transient driver attempts at most `maxNewtonIters` Newton iterations
     * at each timestep before accepting the last iterate (with a warning).
     */
    int maxNewtonIters = 50;

    /**
     * @brief Maximum number of backtracking (under-relaxation) attempts.
     *
     * When a Newton step produces a non-finite or non-improving iterate, the
     * solver successively halves the relaxation factor up to `maxBacktracks`
     * times to try to obtain a finite, reduced-residual iterate.
     */
    int maxBacktracks = 4;

    /** @brief Path to the diagnostic log file written by the solver. */
    std::string diagFile = "diagnostics.log";

    /**
     * @brief Emit verbose diagnostics (element-level dumps) on failures.
     *
     * When true the transient driver will request detailed diagnostic output
     * (per-element `dumpDiagnostics()` calls) in addition to the basic
     * diagnostics written to `diagFile`.
     */
    bool diagVerbose = false;

    /**
     * @brief Maximum allowed absolute magnitude for state variables.
     *
     * This is a defensive guard: candidate iterates producing values whose
     * absolute magnitudes exceed `maxState` will be treated as invalid and
     * rejected by the solver/backtracking logic. The guard helps detect runaway
     * iterates and prevents committing obviously divergent states.
     */
    double maxState = 1e6;

    /**
     * @brief When true, initialize dynamic element states to zero instead of
     *        computing a DC operating point.
     *
     * Useful for quick transient experiments where computing the DC operating
     * point is expensive or undesirable. When `zeroInit` is false the solver
     * will compute the DC operating point to initialize capacitor voltages and
     * inductor currents.
     */
    bool zeroInit = false;

    /**
     * @brief Validate option values and clamp/raise for invalid configuration.
     *
     * This method performs minimal semantic checks on the struct fields and
     * throws `std::invalid_argument` on fatal misconfiguration. Callers should
     * invoke this method after parsing CLI values or before handing the
     * options to the solver.
     *
     * Throws:
     *   - std::invalid_argument if any required relationship is violated
     *     (e.g., negative iteration counts, non-positive maxState).
     */
    void validate() const
    {
        if (maxNewtonIters <= 0)
            throw std::invalid_argument("maxNewtonIters must be > 0");
        if (maxBacktracks < 0)
            throw std::invalid_argument("maxBacktracks must be >= 0");
        if (maxState <= 0.0)
            throw std::invalid_argument("maxState must be > 0");
        // newtonAlpha may be <= 0 to intentionally disable progress in extreme
        // debugging scenarios, so we don't strictly forbid it here.
    }
};
