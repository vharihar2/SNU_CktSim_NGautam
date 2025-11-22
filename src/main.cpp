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
 * @file main.cpp
 *
 * @brief Contains the  implemntation of the main functions
 */

#include "main.hpp"

#include <getopt.h>

#include <iostream>

#include "Solver.hpp"
#include "SolverOptions.hpp"

static void printHelp(const char *prog)
{
    std::cout << "Usage: " << prog << " [options] [netlist-file]\n";
    std::cout << "Options:\n";
    std::cout
        << "  --adaptive                Enable adaptive timestep controller\n";
    std::cout << "  --atol <double>           Absolute tolerance for LTE "
                 "(default 1e-6)\n";
    std::cout << "  --rtol <double>           Relative tolerance for LTE "
                 "(default 1e-3)\n";
    std::cout << "  --safety <double>         Safety factor (default 0.9)\n";
    std::cout << "  --fac-min <double>        Minimum step-change factor "
                 "(default 0.5)\n";
    std::cout << "  --fac-max <double>        Maximum step-change factor "
                 "(default 2.0)\n";
    std::cout << "  --h-min <double>          Minimum timestep (overrides "
                 "internal default)\n";
    std::cout << "  --h-max <double>          Maximum timestep (overrides "
                 "internal default)\n";
    std::cout
        << "  --adaptive-max-retries <int>  Max adaptive retries (default 4)\n";
    std::cout << "  --newton-alpha <double>   Newton under-relaxation alpha "
                 "(default 1.0)\n";
    std::cout << "  --max-newton-iters <int>  Max Newton iterations per step "
                 "(default 50)\n";
    std::cout << "  --max-backtracks <int>    Max backtracks for damping "
                 "(default 4)\n";
    std::cout << "  --diag-file <file>        Diagnostics output file (default "
                 "diagnostics.log)\n";
    std::cout << "  --diag-verbose            Verbose diagnostics\n";
    std::cout << "  --help                    Show this help message\n";
}

int main(int argc, char *argv[])
{
    SolverOptions options;

    static struct option long_options[] = {
        {"adaptive", no_argument, 0, 'a'},
        {"atol", required_argument, 0, 0},
        {"rtol", required_argument, 0, 0},
        {"safety", required_argument, 0, 0},
        {"fac-min", required_argument, 0, 0},
        {"fac-max", required_argument, 0, 0},
        {"h-min", required_argument, 0, 0},
        {"h-max", required_argument, 0, 0},
        {"adaptive-max-retries", required_argument, 0, 0},
        {"newton-alpha", required_argument, 0, 0},
        {"max-newton-iters", required_argument, 0, 0},
        {"max-backtracks", required_argument, 0, 0},
        {"diag-file", required_argument, 0, 0},
        {"diag-verbose", no_argument, 0, 0},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    int option_index = 0;
    int c;
    // Use getopt_long to iterate over options
    while ((c = getopt_long(argc, argv, "ah", long_options, &option_index)) !=
           -1) {
        if (c == 'a') {
            options.enableAdaptive = true;
        } else if (c == 'h') {
            printHelp(argv[0]);
            return 0;
        } else if (c == 0) {
            std::string name = long_options[option_index].name;
            if (name == "atol")
                options.atol = std::stod(optarg);
            else if (name == "rtol")
                options.rtol = std::stod(optarg);
            else if (name == "safety")
                options.safety = std::stod(optarg);
            else if (name == "fac-min")
                options.facMin = std::stod(optarg);
            else if (name == "fac-max")
                options.facMax = std::stod(optarg);
            else if (name == "h-min")
                options.hMin = std::stod(optarg);
            else if (name == "h-max")
                options.hMax = std::stod(optarg);
            else if (name == "adaptive-max-retries")
                options.maxAdaptiveRetries = std::stoi(optarg);
            else if (name == "newton-alpha")
                options.newtonAlpha = std::stod(optarg);
            else if (name == "max-newton-iters")
                options.maxNewtonIters = std::stoi(optarg);
            else if (name == "max-backtracks")
                options.maxBacktracks = std::stoi(optarg);
            else if (name == "diag-file")
                options.diagFile = std::string(optarg);
            else if (name == "diag-verbose")
                options.diagVerbose = true;
        }
    }

    // Remaining non-option args: [netlist-file]
    std::string filename = "circuit.sns";
    if (optind < argc) {
        filename = argv[optind];
    }

    // Validate options (throws on bad input)
    try {
        options.validate();
    } catch (const std::exception &ex) {
        std::cerr << "Invalid solver option: " << ex.what() << std::endl;
        return 1;
    }

    // Prepare argc/argv for solver: program name + filename
    char *new_argv[2];
    new_argv[0] = argv[0];
    new_argv[1] = const_cast<char *>(filename.c_str());

    // Call overloaded runSolver that accepts SolverOptions
    return runSolver(2, new_argv, options);
}
