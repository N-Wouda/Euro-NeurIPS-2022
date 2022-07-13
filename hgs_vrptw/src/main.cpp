#include "CommandLine.h"
#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Split.h"

#include <iostream>

int main(int argc, char *argv[])
try
{
    CommandLine args(argc, argv);
    auto config = args.parse();

    Params params(config);

    Split split(&params);
    LocalSearch ls(&params);
    Population pop(&params, &split, &ls);

    Genetic solver(&params, &split, &pop, &ls);
    auto const res = solver.run(config.nbIter, config.timeLimit);

    if (res.getBestFound())
    {
        auto const *bestSol = res.getBestFound();
        bestSol->exportCVRPLibFormat(config.pathSolution);
    }
}
catch (std::string const &e)
{
    std::cerr << "EXCEPTION | " << e << '\n';
}
catch (std::exception const &e)
{
    std::cerr << "EXCEPTION | " << e.what() << '\n';
}
