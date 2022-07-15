#include "CommandLine.h"
#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"

#include <iostream>

int main(int argc, char *argv[])
try
{
    CommandLine args(argc, argv);
    Params params = args.parse();

    LocalSearch ls(&params);
    Population pop(&params, &ls);

    Genetic solver(&params, &pop, &ls);
    auto const res = solver.run(params.config.nbIter, params.config.timeLimit);

    if (res.getBestFound() != nullptr)
    {
        auto const *bestSol = res.getBestFound();
        bestSol->exportCVRPLibFormat(params.config.pathSolution);
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
