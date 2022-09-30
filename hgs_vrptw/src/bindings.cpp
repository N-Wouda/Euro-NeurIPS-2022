#include "Config.h"
#include "Exchange.h"
#include "GeneticAlgorithm.h"
#include "Individual.h"
#include "LocalSearch.h"
#include "LocalSearchOperator.h"
#include "MaxIterations.h"
#include "MaxRuntime.h"
#include "MoveTwoClientsReversed.h"
#include "Params.h"
#include "Population.h"
#include "RelocateStar.h"
#include "Result.h"
#include "Statistics.h"
#include "StoppingCriterion.h"
#include "SwapStar.h"
#include "TwoOpt.h"
#include "XorShift128.h"
#include "crossover.h"

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(hgspy, m)
{
    py::class_<XorShift128>(m, "XorShift128")
        .def(py::init<int>(), py::arg("seed"));

    py::class_<Individual>(m, "Individual")
        .def(py::init<Params *, XorShift128 *>(),
             py::arg("params"),
             py::arg("rng"))
        .def(py::init<Params *, std::vector<std::vector<int>>>(),
             py::arg("params"),
             py::arg("routes"))
        .def("cost", &Individual::cost)
        .def("get_routes", &Individual::getRoutes)
        .def("get_neighbours", &Individual::getNeighbours)
        .def("is_feasible", &Individual::isFeasible)
        .def("has_excess_capacity", &Individual::hasExcessCapacity)
        .def("has_time_warp", &Individual::hasTimeWarp)
        .def("export_cvrplib_format", &Individual::exportCVRPLibFormat);

    py::class_<LocalSearch>(m, "LocalSearch")
        .def(py::init<Params &, XorShift128 &>(),
             py::arg("params"),
             py::arg("rng"))
        .def("add_node_operator",
             static_cast<void (LocalSearch::*)(LocalSearchOperator<Node> &)>(
                 &LocalSearch::addNodeOperator),
             py::arg("op"))
        .def("add_route_operator",
             static_cast<void (LocalSearch::*)(LocalSearchOperator<Route> &)>(
                 &LocalSearch::addRouteOperator),
             py::arg("op"))
        .def("__call__", &LocalSearch::operator(), py::arg("indiv"));

    py::class_<Config>(m, "Config")
        .def(py::init<int,
                      size_t,
                      int,
                      bool,
                      size_t,
                      size_t,
                      double,
                      double,
                      double,
                      size_t,
                      size_t,
                      size_t,
                      size_t,
                      double,
                      size_t,
                      size_t,
                      size_t,
                      int,
                      size_t,
                      int,
                      int,
                      size_t,
                      int,
                      int,
                      size_t>(),
             py::arg("seed") = 0,
             py::arg("nbIter") = 10'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("collectStatistics") = false,
             py::arg("initialTimeWarpPenalty") = 1,
             py::arg("nbPenaltyManagement") = 100,
             py::arg("feasBooster") = 2.,
             py::arg("penaltyIncrease") = 1.2,
             py::arg("penaltyDecrease") = 0.85,
             py::arg("minPopSize") = 25,
             py::arg("generationSize") = 40,
             py::arg("nbElite") = 4,
             py::arg("nbClose") = 5,
             py::arg("targetFeasible") = 0.2,
             py::arg("repairProbability") = 50,
             py::arg("repairBooster") = 10,
             py::arg("selectProbability") = 90,
             py::arg("nbVeh") = INT_MAX,
             py::arg("nbGranular") = 40,
             py::arg("weightWaitTime") = 2,
             py::arg("weightTimeWarp") = 10,
             py::arg("intensificationProbability") = 0,
             py::arg("circleSectorOverlapToleranceDegrees") = 0,
             py::arg("minCircleSectorSizeDegrees") = 15,
             py::arg("destroyPct") = 20)
        .def_readonly("seed", &Config::seed)
        .def_readonly("nbIter", &Config::nbIter)
        .def_readonly("timeLimit", &Config::timeLimit)
        .def_readonly("collectStatistics", &Config::collectStatistics)
        .def_readonly("initialTimeWarpPenalty", &Config::initialTimeWarpPenalty)
        .def_readonly("nbPenaltyManagement", &Config::nbPenaltyManagement)
        .def_readonly("feasBooster", &Config::feasBooster)
        .def_readonly("penaltyIncrease", &Config::penaltyIncrease)
        .def_readonly("penaltyDecrease", &Config::penaltyDecrease)
        .def_readonly("minPopSize", &Config::minPopSize)
        .def_readonly("generationSize", &Config::generationSize)
        .def_readonly("nbElite", &Config::nbElite)
        .def_readonly("nbClose", &Config::nbClose)
        .def_readonly("targetFeasible", &Config::targetFeasible)
        .def_readonly("repairProbability", &Config::repairProbability)
        .def_readonly("repairBooster", &Config::repairBooster)
        .def_readonly("selectProbability", &Config::selectProbability)
        .def_readonly("nbVeh", &Config::nbVeh)
        .def_readonly("nbGranular", &Config::nbGranular)
        .def_readonly("weightWaitTime", &Config::weightWaitTime)
        .def_readonly("weightTimeWarp", &Config::weightTimeWarp)
        .def_readonly("circleSectorOverlapTolerance",
                      &Config::circleSectorOverlapTolerance)
        .def_readonly("minCircleSectorSize", &Config::minCircleSectorSize)
        .def_readonly("destroyPct", &Config::destroyPct);

    py::class_<Params>(m, "Params")
        .def(py::init<Config const &,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      int,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      std::vector<std::vector<int>> const &,
                      std::vector<int> const &>(),
             py::arg("config"),
             py::arg("coords"),
             py::arg("demands"),
             py::arg("vehicle_cap"),
             py::arg("time_windows"),
             py::arg("service_durations"),
             py::arg("duration_matrix"),
             py::arg("release_times"));

    py::class_<Population>(m, "Population")
        .def(py::init<Params &, XorShift128 &>(),
             py::arg("params"),
             py::arg("rng"));

    py::class_<Statistics>(m, "Statistics")
        .def("num_iters", &Statistics::numIters)
        .def("run_times", &Statistics::runTimes)
        .def("iter_times", &Statistics::iterTimes)
        .def("pop_sizes", &Statistics::popSizes)
        .def("num_feasible_pop", &Statistics::numFeasiblePop)
        .def("feas_diversity", &Statistics::feasDiversity)
        .def("feas_best", &Statistics::feasBest)
        .def("feas_average", &Statistics::feasAverage)
        .def("infeas_diversity", &Statistics::infeasDiversity)
        .def("infeas_best", &Statistics::infeasBest)
        .def("infeas_average", &Statistics::infeasAverage)
        .def("penalties_capacity", &Statistics::penaltiesCapacity)
        .def("penalties_time_warp", &Statistics::penaltiesTimeWarp)
        .def("incumbents", &Statistics::incumbents)
        .def("to_csv",
             &Statistics::toCsv,
             py::arg("path"),
             py::arg("sep") = ',');

    py::class_<Result>(m, "Result")
        .def("get_best_found",
             &Result::getBestFound,
             py::return_value_policy::reference)
        .def("get_statistics",
             &Result::getStatistics,
             py::return_value_policy::reference)
        .def("get_iterations",
             &Result::getIterations,
             py::return_value_policy::reference)
        .def("get_run_time",
             &Result::getRunTime,
             py::return_value_policy::reference);

    py::class_<GeneticAlgorithm>(m, "GeneticAlgorithm")
        .def(py::init<Params &, XorShift128 &, Population &, LocalSearch &>(),
             py::arg("params"),
             py::arg("rng"),
             py::arg("population"),
             py::arg("local_search"))
        .def("add_crossover_operator",
             &GeneticAlgorithm::addCrossoverOperator,
             py::arg("op"))
        .def("run", &GeneticAlgorithm::run, py::arg("stop"));

    // Stopping criteria (as a submodule)
    py::module stop = m.def_submodule("stop");

    py::class_<StoppingCriterion>(stop, "StoppingCriterion");

    py::class_<MaxIterations, StoppingCriterion>(stop, "MaxIterations")
        .def(py::init<size_t>(), py::arg("max_iterations"));

    py::class_<MaxRuntime, StoppingCriterion>(stop, "MaxRuntime")
        .def(py::init<size_t>(), py::arg("max_runtime"));

    // Crossover operators (as a submodule)
    py::module xOps = m.def_submodule("crossover");

    xOps.def("broken_pairs_exchange", &brokenPairsExchange);
    xOps.def("selective_route_exchange", &selectiveRouteExchange);

    // Local search operators (as a submodule)
    py::module lsOps = m.def_submodule("operators");

    py::class_<LocalSearchOperator<Node>>(lsOps, "NodeLocalSearchOperator");
    py::class_<LocalSearchOperator<Route>>(lsOps, "RouteLocalSearchOperator");

    py::class_<Exchange<1, 0>, LocalSearchOperator<Node>>(lsOps, "Exchange10")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<2, 0>, LocalSearchOperator<Node>>(lsOps, "Exchange20")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<3, 0>, LocalSearchOperator<Node>>(lsOps, "Exchange30")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<1, 1>, LocalSearchOperator<Node>>(lsOps, "Exchange11")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<2, 1>, LocalSearchOperator<Node>>(lsOps, "Exchange21")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<3, 1>, LocalSearchOperator<Node>>(lsOps, "Exchange31")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<2, 2>, LocalSearchOperator<Node>>(lsOps, "Exchange22")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<3, 2>, LocalSearchOperator<Node>>(lsOps, "Exchange32")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<Exchange<3, 3>, LocalSearchOperator<Node>>(lsOps, "Exchange33")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<MoveTwoClientsReversed, LocalSearchOperator<Node>>(
        lsOps, "MoveTwoClientsReversed")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<TwoOpt, LocalSearchOperator<Node>>(lsOps, "TwoOpt")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<RelocateStar, LocalSearchOperator<Route>>(lsOps, "RelocateStar")
        .def(py::init<Params const &>(), py::arg("params"));

    py::class_<SwapStar, LocalSearchOperator<Route>>(lsOps, "SwapStar")
        .def(py::init<Params const &>(), py::arg("params"));
}
