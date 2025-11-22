/*
 * SolverOptions.hpp
 *
 * Defines the SolverOptions struct used to carry solver and adaptive
 * controller knobs from the CLI into the solver.
 */
#pragma once

#include <string>
#include <stdexcept>

struct SolverOptions
{
    // Adaptive timestep controller
    bool enableAdaptive = false;
    double atol = 1e-6;
    double rtol = 1e-3;
    double safety = 0.9;
    double facMin = 0.5;
    double facMax = 2.0;
    double hMin = 0.0; // 0 means use internal default (must be validated)
    double hMax = 0.0; // 0 means use internal default (must be validated)
    int maxAdaptiveRetries = 4;

    // Newton solver / damping
    double newtonAlpha = 1.0;
    int maxNewtonIters = 50;
    int maxBacktracks = 4;

    // Growth policy
    int growthSuccessCount = 5;

    // Diagnostics
    std::string diagFile = "diagnostics.log";
    bool diagVerbose = false;
    // Maximum allowed (absolute) magnitude for state variables. If a
    // candidate solution produces values larger than this, the adaptive
    // controller will reject the step to avoid committing divergent states.
    double maxState = 1e6;

    // Validate and clamp obvious invalid values. Throws std::invalid_argument
    // on fatal configuration errors.
    void validate() const
    {
        if (atol < 0.0) throw std::invalid_argument("atol must be >= 0");
        if (rtol < 0.0) throw std::invalid_argument("rtol must be >= 0");
        if (safety <= 0.0) throw std::invalid_argument("safety must be > 0");
        if (!(facMin > 0.0 && facMin <= facMax))
            throw std::invalid_argument("facMin must be >0 and <= facMax");
        if (maxNewtonIters <= 0) throw std::invalid_argument("maxNewtonIters must be >0");
        if (maxBacktracks < 0) throw std::invalid_argument("maxBacktracks must be >=0");
        if (maxAdaptiveRetries < 0) throw std::invalid_argument("maxAdaptiveRetries must be >=0");
        if (maxState <= 0.0) throw std::invalid_argument("maxState must be > 0");
    }
};
