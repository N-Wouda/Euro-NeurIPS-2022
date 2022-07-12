#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Split.h"
#include "commandline.h"

#include <iostream>

int main(int argc, char *argv[])
try
{
    CommandLine args(argc, argv);
    Params params(args);

    Split split(&params);
    LocalSearch ls(&params);
    Population pop(&params, &split, &ls);

    Genetic solver(&params, &split, &pop, &ls);
    solver.run(args.config.nbIter, args.config.timeLimit);

    if (pop.getBestFound() != nullptr)
    {
        pop.getBestFound()->exportCVRPLibFormat(args.config.pathSolution);

        pop.exportSearchProgress(args.config.pathSolution + ".PG.csv",
                                 args.config.pathInstance,
                                 args.config.seed);

        if (!args.config.pathBKS.empty())
            pop.exportBKS(args.config.pathBKS);
    }
}
catch (std::string const &e)
{
    std::cout << "EXCEPTION | " << e << '\n';
}
catch (std::exception const &e)
{
    std::cout << "EXCEPTION | " << e.what() << '\n';
}
