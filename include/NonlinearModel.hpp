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
        double res = 0.0;
        double pow_u = 1.0;
        for (double a : m_coeffs) {
            res += a * pow_u;
            pow_u *= u;
        }
        return res;
    }

    double dqdu(double u) const override
    {
        // derivative: sum_{k>=1} k * a_k * u^{k-1}
        double res = 0.0;
        double pow_u = 1.0;  // u^{k-1}
        for (size_t k = 1; k < m_coeffs.size(); ++k) {
            res += static_cast<double>(k) * m_coeffs[k] * pow_u;
            pow_u *= u;
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
