/*
 * SolverOptions.hpp
 *
 * Defines the SolverOptions struct used to carry solver configuration
 * knobs from the CLI into the solver.
 */
#pragma once

#include <stdexcept>
#include <string>

struct SolverOptions
{
    // Newton solver / damping
    double newtonAlpha = 1.0;
    int maxNewtonIters = 50;
    int maxBacktracks = 4;
    // Diagnostics
    std::string diagFile = "diagnostics.log";
    bool diagVerbose = false;
    // Maximum allowed (absolute) magnitude for state variables. If a
    // candidate solution produces values larger than this, the solver will
    // treat the candidate as invalid to avoid committing divergent states.
    double maxState = 1e6;
    // When true, skip the DC operating point initialization and start all
    // dynamic element states (capacitor voltages / inductor currents) at
    // zero. Useful for quick transient tests where DC initialization is
    // undesirable.
    bool zeroInit = false;

    // Validate and clamp obvious invalid values. Throws std::invalid_argument
    // on fatal configuration errors.
    void validate() const
    {
        if (maxNewtonIters <= 0)
            throw std::invalid_argument("maxNewtonIters must be >0");
        if (maxBacktracks < 0)
            throw std::invalid_argument("maxBacktracks must be >=0");
        if (maxState <= 0.0)
            throw std::invalid_argument("maxState must be > 0");
    }
};
