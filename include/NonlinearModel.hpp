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
 * @brief Base abstraction for nonlinear constitutive models.
 *
 * The simulator will call the appropriate methods depending on whether the
 * element is charge-based (capacitor) or flux-based (inductor). Default
 * implementations return 0.0 so linear elements may ignore the model.
 */
class NonlinearModel
{
   public:
    virtual ~NonlinearModel() = default;

    // Charge-based API (for capacitors)
    virtual double q(double /*u*/) const { return 0.0; }
    virtual double dqdu(double /*u*/) const { return 0.0; }

    // Flux-based API (for inductors)
    virtual double phi(double /*i*/) const { return 0.0; }
    virtual double dphidi(double /*i*/) const { return 0.0; }
};

/**
 * @brief Polynomial charge model: q(u) = a0 + a1*u + a2*u^2 + ...
 */
class PolynomialChargeModel : public NonlinearModel
{
   public:
    explicit PolynomialChargeModel(const std::vector<double> &coeffs)
        : m_coeffs(coeffs)
    {
    }

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
 */
class PolynomialFluxModel : public NonlinearModel
{
   public:
    explicit PolynomialFluxModel(const std::vector<double> &coeffs)
        : m_coeffs(coeffs)
    {
    }

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
 * @brief Factory helpers
 */
inline std::shared_ptr<NonlinearModel> makePolynomialChargeModel(
    const std::vector<double> &coeffs)
{
    return std::make_shared<PolynomialChargeModel>(coeffs);
}

inline std::shared_ptr<NonlinearModel> makePolynomialFluxModel(
    const std::vector<double> &coeffs)
{
    return std::make_shared<PolynomialFluxModel>(coeffs);
}
