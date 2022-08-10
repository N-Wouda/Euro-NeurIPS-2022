#include "CommandLine.h"
#include "GeneticAlgorithm.h"
#include "LocalSearch.h"
#include "MaxRuntime.h"
#include "MoveSingleClient.h"
#include "MoveTwoClients.h"
#include "MoveTwoClientsReversed.h"
#include "Params.h"
#include "Population.h"
#include "RelocateStar.h"
#include "SwapStar.h"
#include "SwapTwoClientPairs.h"
#include "SwapTwoClientsForOne.h"
#include "SwapTwoSingleClients.h"
#include "TwoOpt.h"
#include "XorShift128.h"
#include "crossover.h"

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
    Population pop(params, rng);

    LocalSearch ls(params, rng);

    auto moveSingle = MoveSingleClient();
    ls.addNodeOperator(moveSingle);

    auto moveTwo = MoveTwoClients();
    ls.addNodeOperator(moveTwo);

    auto moveTwoReverse = MoveTwoClientsReversed();
    ls.addNodeOperator(moveTwoReverse);

    auto swapTwoPairs = SwapTwoClientPairs();
    ls.addNodeOperator(swapTwoPairs);

    auto swapTwoForOne = SwapTwoClientsForOne();
    ls.addNodeOperator(swapTwoForOne);

    auto swapSingle = SwapTwoSingleClients();
    ls.addNodeOperator(swapSingle);

    auto twoOpt = TwoOpt();
    ls.addNodeOperator(twoOpt);

    auto relocateStar = RelocateStar();
    ls.addRouteOperator(relocateStar);

    auto swapStar = SwapStar();
    ls.addRouteOperator(swapStar);

    GeneticAlgorithm solver(params, rng, pop, ls);
    solver.addCrossoverOperator(orderedExchange);
    solver.addCrossoverOperator(selectiveRouteExchange);

    MaxRuntime stop(config.timeLimit);
    auto const res = solver.run(stop);

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
