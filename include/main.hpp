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

/**
 * @file main.hpp
 * @brief Declarations and documentation for the program entry point.
 *
 * This header documents the program's top-level driver (implemented in
 * src/main.cpp). The implementation in `main.cpp` parses command-line
 * options, validates solver options, selects the input netlist filename and
 * dispatches to the solver entrypoint.
 *
 * Primary responsibilities of the top-level driver:
 *  - Parse long and short CLI options (see SolverOptions for available flags)
 *  - Validate options and report errors to stderr
 *  - Determine the netlist filename (default: \"circuit.sns\") when not
 *    supplied on the command line
 *  - Call into the solver entrypoint (e.g., `runSolver`) with prepared
 *    arguments and `SolverOptions`
 *
 * Implementation notes:
 *  - `main.cpp` provides a small internal helper `printHelp(const char*)`
 *    to print usage information. That helper is implementation-local and not
 *    exposed via this header.
 *  - The authoritative solver entrypoints are declared in `include/Solver.hpp`.
 *
 * Example usage:
 *   ./SNU_Spice --newton-alpha 0.8 --diag-verbose my_circuit.sns
 *
 * See also:
 *  - SolverOptions (include/SolverOptions.hpp) for CLI-configurable options
 *  - runSolver (include/Solver.hpp) which the driver invokes to run the solver
 */

#pragma once

// This header intentionally does not expose additional symbols. It exists to
// provide a stable place for file-level documentation about the program entry
// point and to be included by the implementation file (src/main.cpp).
