#include "CommandLine.h"
#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "XorShift128.h"

#include <iostream>

int main(int argc, char *argv[])
try
{
    CommandLine args(argc, argv);
    auto config = args.parse();
    auto rng = XorShift128(config.seed);
    auto params = Params(config, args.instPath());

    LocalSearch ls(params, rng);
    Population pop(params, rng, ls);

    Genetic solver(params, rng, pop, ls);
    auto const res = solver.run();

    if (res.getBestFound() != nullptr)
    {
        auto const *bestSol = res.getBestFound();
        bestSol->exportCVRPLibFormat(args.solPath());
    }
}
catch (std::exception const &e)
{
    std::cerr << "EXCEPTION | " << e.what() << '\n';
}
catch (...)
{
    std::cerr << "UNKNOWN EXCEPTION\n";
}
