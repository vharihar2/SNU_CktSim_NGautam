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
/*
 * NonlinearModel.hpp
 *
 * Small header-only abstraction for nonlinear constitutive models used by
 * nonlinear capacitors and inductors. Provides a minimal API for charge/flux
 * models and two simple polynomial implementations (charge and flux).
 */

#pragma once

#include <cmath>
#include <memory>
#include <vector>

/**
 * @file NonlinearModel.hpp
 * @brief Abstractions and simple implementations for nonlinear constitutive models.
 *
 * This header declares a lightweight polymorphic interface, `NonlinearModel`,
 * used to represent nonlinear constitutive relations for energy-storage
 * elements in the simulator:
 *
 * - Charge-based models (for nonlinear capacitors) implement q(u) and its
 *   derivative dq/du where `u` is the voltage across the element and `q` is
 *   the stored charge.
 *
 * - Flux-based models (for nonlinear inductors) implement phi(i) and its
 *   derivative dphi/di where `i` is the element current and `phi` is the
 *   magnetic flux linkage.
 *
 * Two concrete, header-only polynomial models are provided:
 *
 * - `PolynomialChargeModel` evaluates q(u) = a0 + a1*u + a2*u^2 + ... using a
 *   Horner-like accumulation with intermediate clamping to avoid producing
 *   Inf/NaN values for extreme inputs.
 *
 * - `PolynomialFluxModel` evaluates phi(i) = b0 + b1*i + b2*i^2 + ... and its
 *   derivative using straightforward accumulation.
 *
 * The simulator code calls the appropriate API depending on whether an
 * element is charge- or flux-based. Default implementations in the base class
 * return 0.0 so linear elements that do not use a nonlinear model can ignore
 * it.
 *
 * Usage example:
 * @code
 * auto model = makePolynomialChargeModel({0.0, 1e-6}); // q(u) = a1*u (linear C=1e-6)
 * double charge = model->q(5.0);      // q at 5 V
 * double dqdu  = model->dqdu(5.0);    // incremental capacitance at 5 V
 * @endcode
 */

/**
 * @brief Base abstraction for nonlinear constitutive models.
 *
 * Implementations provide either a charge-based API (for capacitors) or a
 * flux-based API (for inductors). The simulator will invoke only the
 * methods relevant to the element type; the default implementations return
 * 0.0 so elements without an attached nonlinear model can safely ignore it.
 */
class NonlinearModel
{
   public:
    virtual ~NonlinearModel() = default;

    /**
     * @brief Charge as a function of voltage: q(u).
     *
     * For a charge-based constitutive model (nonlinear capacitor), return the
     * stored charge associated with the voltage `u` across the element.
     *
     * Default implementation returns 0.0.
     *
     * @param u Voltage across the element (V).
     * @return Charge (C).
     */
    virtual double q(double /*u*/) const { return 0.0; }

    /**
     * @brief Derivative of charge with respect to voltage: dq/du.
     *
     * Returns the incremental capacitance at the operating point `u`. This is
     * used by linearization (Newton) and transient companion computations.
     *
     * Default implementation returns 0.0.
     *
     * @param u Voltage across the element (V).
     * @return dq/du (F).
     */
    virtual double dqdu(double /*u*/) const { return 0.0; }

    /**
     * @brief Flux as a function of current: phi(i).
     *
     * For a flux-based constitutive model (nonlinear inductor), return the
     * flux linkage corresponding to current `i`.
     *
     * Default implementation returns 0.0.
     *
     * @param i Current through the element (A).
     * @return Flux linkage (Wb or V·s).
     */
    virtual double phi(double /*i*/) const { return 0.0; }

    /**
     * @brief Derivative of flux with respect to current: dphi/di.
     *
     * Returns the incremental inductance at the operating point `i`. Used for
     * linearization and companion model formation.
     *
     * Default implementation returns 0.0.
     *
     * @param i Current through the element (A).
     * @return dphi/di (H).
     */
    virtual double dphidi(double /*i*/) const { return 0.0; }
};

/**
 * @brief Polynomial charge model: q(u) = a0 + a1*u + a2*u^2 + ...
 *
 * The polynomial coefficients are provided in ascending order (a0, a1, a2, ...).
 * Evaluation uses a Horner-like reverse accumulation which is numerically
 * more stable and avoids repeated calls to `pow()`. To improve robustness the
 * implementation performs intermediate overflow checks and clamps outputs to a
 * large but finite sentinel `MAX_POLY_OUT` to avoid producing Inf/NaN values
 * that would destabilize the nonlinear solver.
 */
class PolynomialChargeModel : public NonlinearModel
{
   public:
    /**
     * @brief Construct a polynomial charge model.
     *
     * @param coeffs Polynomial coefficients {a0, a1, a2, ...}.
     */
    explicit PolynomialChargeModel(const std::vector<double> &coeffs)
        : m_coeffs(coeffs)
    {
    }

    /**
     * @brief Evaluate q(u) using Horner-like accumulation with clamping.
     *
     * @param u Voltage across the element (V).
     * @return Charge q(u) (C), clamped to a large finite sentinel for safety.
     */
    double q(double u) const override
    {
        // Horner's method is numerically more stable and avoids
        // repeated pow() growth which can overflow quickly for large u.
        // Also clamp outputs to avoid producing Inf/NaN that destabilize
        // the solver. Use a very large but finite clamp bound.
        // Reduce clamp to a more conservative magnitude and avoid
        // intermediate overflow by checking before multiplication.
        const double MAX_POLY_OUT = 1e30;
        double res = 0.0;
        for (auto it = m_coeffs.rbegin(); it != m_coeffs.rend(); ++it) {
            // Prevent intermediate overflow: if |res| * |u| would exceed
            // MAX_POLY_OUT, clamp and stop.
            double abs_u = std::abs(u);
            double safe_limit = MAX_POLY_OUT / std::max(1.0, abs_u);
            if (std::abs(res) > safe_limit) {
                res = (res > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            double next = res * u + *it;
            if (!std::isfinite(next)) {
                // clamp to finite sentinel and break
                res = (next > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            if (std::abs(next) > MAX_POLY_OUT) {
                res = (next > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            res = next;
        }
        if (!std::isfinite(res)) {
            // Return a large finite sentinel value instead of Inf/NaN
            return (res > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
        }
        return res;
    }

    /**
     * @brief Evaluate derivative dq/du for the polynomial model.
     *
     * Computes the polynomial derivative using a Horner-like evaluation:
     * d/du (a0 + a1 u + a2 u^2 + ...) = a1 + 2*a2*u + 3*a3*u^2 + ...
     *
     * The implementation accumulates from the highest degree down to 1 and
     * performs the same intermediate safety checks as `q()` to avoid overflow.
     *
     * @param u Voltage across the element (V).
     * @return dq/du (F), clamped to a large finite sentinel if necessary.
     */
    double dqdu(double u) const override
    {
        // Compute derivative using Horner-like evaluation for the
        // polynomial derivative: d/du (a0 + a1 u + a2 u^2 + ...) =
        // a1 + 2*a2*u + 3*a3*u^2 + ...
        const double MAX_POLY_OUT = 1e30;
        double res = 0.0;
        if (m_coeffs.size() <= 1) return 0.0;
        // Evaluate derivative via Horner-like accumulation from highest
        // degree down to 1. Use safe multiply checks to avoid overflow.
        for (int kk = static_cast<int>(m_coeffs.size()) - 1; kk >= 1; --kk) {
            double coeff = static_cast<double>(kk) * m_coeffs[kk];
            double abs_u = std::abs(u);
            double safe_limit = MAX_POLY_OUT / std::max(1.0, abs_u);
            if (std::abs(res) > safe_limit) {
                res = (res > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            double next = res * u + coeff;
            if (!std::isfinite(next)) {
                res = (next > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            if (std::abs(next) > MAX_POLY_OUT) {
                res = (next > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
                break;
            }
            res = next;
        }
        if (!std::isfinite(res)) {
            return (res > 0.0) ? MAX_POLY_OUT : -MAX_POLY_OUT;
        }
        return res;
    }

   private:
    std::vector<double> m_coeffs;
};

/**
 * @brief Polynomial flux model: phi(i) = b0 + b1*i + b2*i^2 + ...
 *
 * Coefficients are provided in ascending order (b0, b1, b2, ...). This class
 * provides straightforward evaluation for phi(i) and its derivative dphi/di.
 * Unlike the charge model, evaluation here uses a simple forward accumulation
 * which is adequate for moderate polynomials; if needed it can be replaced by
 * a Horner-style implementation similar to `PolynomialChargeModel`.
 */
class PolynomialFluxModel : public NonlinearModel
{
   public:
    /**
     * @brief Construct a polynomial flux model.
     *
     * @param coeffs Polynomial coefficients {b0, b1, b2, ...}.
     */
    explicit PolynomialFluxModel(const std::vector<double> &coeffs)
        : m_coeffs(coeffs)
    {
    }

    /**
     * @brief Evaluate phi(i).
     *
     * @param i Current through the element (A).
     * @return Flux linkage phi(i) (Wb or V·s).
     */
    double phi(double i) const override
    {
        double res = 0.0;
        double pow_i = 1.0;
        for (double b : m_coeffs) {
            res += b * pow_i;
            pow_i *= i;
        }
        return res;
    }

    /**
     * @brief Evaluate derivative dphi/di.
     *
     * Computes the derivative: dphi/di = b1 + 2*b2*i + 3*b3*i^2 + ...
     *
     * @param i Current through the element (A).
     * @return dphi/di (H).
     */
    double dphidi(double i) const override
    {
        double res = 0.0;
        double pow_i = 1.0;  // i^{k-1}
        for (size_t k = 1; k < m_coeffs.size(); ++k) {
            res += static_cast<double>(k) * m_coeffs[k] * pow_i;
            pow_i *= i;
        }
        return res;
    }

   private:
    std::vector<double> m_coeffs;
};

/**
 * @brief Factory helper: create a polynomial charge model.
 *
 * Convenience function returning a shared pointer to `PolynomialChargeModel`.
 *
 * @param coeffs Polynomial coefficients {a0, a1, a2, ...}.
 * @return std::shared_ptr<NonlinearModel> owning the created model.
 */
inline std::shared_ptr<NonlinearModel> makePolynomialChargeModel(
    const std::vector<double> &coeffs)
{
    return std::make_shared<PolynomialChargeModel>(coeffs);
}

/**
 * @brief Factory helper: create a polynomial flux model.
 *
 * Convenience function returning a shared pointer to `PolynomialFluxModel`.
 *
 * @param coeffs Polynomial coefficients {b0, b1, b2, ...}.
 * @return std::shared_ptr<NonlinearModel> owning the created model.
 */
inline std::shared_ptr<NonlinearModel> makePolynomialFluxModel(
    const std::vector<double> &coeffs)
{
    return std::make_shared<PolynomialFluxModel>(coeffs);
}
