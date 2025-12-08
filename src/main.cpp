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
    std::cout << "  --newton-alpha <double>   Newton under-relaxation alpha "
                 "(default 1.0)\n";
    std::cout << "  --max-newton-iters <int>  Max Newton iterations per step "
                 "(default 50)\n";
    std::cout << "  --max-backtracks <int>    Max backtracks for damping "
                 "(default 4)\n";
    std::cout << "  --diag-file <file>        Diagnostics output file (default "
                 "diagnostics.log)\n";
    std::cout << "  --diag-verbose            Verbose diagnostics\n";
    std::cout
        << "  --zero-init               Skip DC init; zero dynamic states\n";
    std::cout << "  --help                    Show this help message\n";
}

int main(int argc, char *argv[])
{
    SolverOptions options;

    static struct option long_options[] = {
        {"newton-alpha", required_argument, 0, 0},
        {"max-newton-iters", required_argument, 0, 0},
        {"max-backtracks", required_argument, 0, 0},
        {"diag-file", required_argument, 0, 0},
        {"diag-verbose", no_argument, 0, 0},
        {"zero-init", no_argument, 0, 0},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    int option_index = 0;
    int c;
    // Use getopt_long to iterate over options
    while ((c = getopt_long(argc, argv, "h", long_options, &option_index)) !=
           -1) {
        if (c == 'h') {
            printHelp(argv[0]);
            return 0;
        } else if (c == 0) {
            std::string name = long_options[option_index].name;
            if (name == "newton-alpha")
                options.newtonAlpha = std::stod(optarg);
            else if (name == "max-newton-iters")
                options.maxNewtonIters = std::stoi(optarg);
            else if (name == "max-backtracks")
                options.maxBacktracks = std::stoi(optarg);
            else if (name == "diag-file")
                options.diagFile = std::string(optarg);
            else if (name == "diag-verbose")
                options.diagVerbose = true;
            else if (name == "zero-init")
                options.zeroInit = true;
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
