#include "CommandLine.h"
#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "XorShift128.h"

#include <chrono>
#include <iostream>

int main(int argc, char **argv)
try
{
    using clock = std::chrono::system_clock;
    auto start = clock::now();

    CommandLine args(argc, argv);
    auto config = args.parse();

    XorShift128 rng(config.seed);
    Params params(config, args.instPath());
    LocalSearch ls(params, rng);
    Population pop(params, rng, ls);

    Genetic solver(params, rng, pop);

    auto const until = start + std::chrono::seconds(config.timeLimit);
    auto const res = solver.runUntil(until);

    std::chrono::duration<double> const timeDelta = clock::now() - start;
    auto const &bestSol = res.getBestFound();
    bestSol.exportCVRPLibFormat(args.solPath(), timeDelta.count());
}
catch (std::exception const &e)
{
    std::cerr << "EXCEPTION | " << e.what() << '\n';
}
catch (...)
{
    std::cerr << "UNKNOWN EXCEPTION\n";
}
