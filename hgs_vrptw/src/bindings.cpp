#include "Config.h"
#include "GeneticAlgorithm.h"
#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Statistics.h"
#include "XorShift128.h"
#include "crossover.h"
#include "operators.h"

#include "MaxIterations.h"
#include "MaxRuntime.h"
#include "StoppingCriterion.h"

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
        .def(py::init<Params *, std::vector<int>>(),
             py::arg("params"),
             py::arg("tour"))
        .def(py::init<Params *, std::vector<std::vector<int>>>(),
             py::arg("params"),
             py::arg("routes"))
        .def("cost", &Individual::cost)
        .def("get_routes", &Individual::getRoutes)
        .def("get_tour", &Individual::getTour)
        .def("is_feasible", &Individual::isFeasible)
        .def("has_excess_capacity", &Individual::hasExcessCapacity)
        .def("has_time_warp", &Individual::hasTimeWarp);

    py::class_<LocalSearch>(m, "LocalSearch")
        .def(py::init<Params &, XorShift128 &>(),
             py::arg("params"),
             py::arg("rng"))
        .def("add_node_operator", &LocalSearch::addNodeOperator, py::arg("op"))
        .def(
            "add_route_operator", &LocalSearch::addRouteOperator, py::arg("op"))
        .def("__call__",
             &LocalSearch::operator(),
             py::arg("indiv"),
             py::arg("excessCapacityPenalty"),
             py::arg("timeWarpPenalty"));

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
                      double,
                      int,
                      size_t,
                      int,
                      int,
                      int,
                      int,
                      int>(),
             py::arg("seed") = 0,
             py::arg("nbIter") = 20'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("collectStatistics") = false,
             py::arg("initialTimeWarpPenalty") = 1,
             py::arg("nbPenaltyManagement") = 100,
             py::arg("feasBooster") = 2.,
             py::arg("penaltyIncrease") = 1.2,
             py::arg("penaltyDecrease") = 0.85,
             py::arg("minimumPopulationSize") = 25,
             py::arg("generationSize") = 40,
             py::arg("nbElite") = 4,
             py::arg("nbClose") = 5,
             py::arg("targetFeasible") = 0.2,
             py::arg("repairProbability") = 50,
             py::arg("repairBooster") = 10,
             py::arg("selectProbability") = 90,
             py::arg("diversityWeight") = 0.,
             py::arg("nbVeh") = INT_MAX,
             py::arg("nbGranular") = 40,
             py::arg("weightWaitTime") = 2,
             py::arg("weightTimeWarp") = 10,
             py::arg("intensificationProbability") = 15,
             py::arg("circleSectorOverlapToleranceDegrees") = 0,
             py::arg("minCircleSectorSizeDegrees") = 15)
        .def_readonly("seed", &Config::seed)
        .def_readonly("nbIter", &Config::nbIter)
        .def_readonly("timeLimit", &Config::timeLimit)
        .def_readonly("collectStatistics", &Config::collectStatistics)
        .def_readonly("initialTimeWarpPenalty", &Config::initialTimeWarpPenalty)
        .def_readonly("nbPenaltyManagement", &Config::nbPenaltyManagement)
        .def_readonly("feasBooster", &Config::feasBooster)
        .def_readonly("penaltyIncrease", &Config::penaltyIncrease)
        .def_readonly("penaltyDecrease", &Config::penaltyDecrease)
        .def_readonly("minimumPopulationSize", &Config::minimumPopulationSize)
        .def_readonly("generationSize", &Config::generationSize)
        .def_readonly("nbElite", &Config::nbElite)
        .def_readonly("nbClose", &Config::nbClose)
        .def_readonly("targetFeasible", &Config::targetFeasible)
        .def_readonly("repairProbability", &Config::repairProbability)
        .def_readonly("repairBooster", &Config::repairBooster)
        .def_readonly("selectProbability", &Config::selectProbability)
        .def_readonly("diversityWeight", &Config::diversityWeight)
        .def_readonly("nbVeh", &Config::nbVeh)
        .def_readonly("nbGranular", &Config::nbGranular)
        .def_readonly("weightWaitTime", &Config::weightWaitTime)
        .def_readonly("weightTimeWarp", &Config::weightTimeWarp)
        .def_readonly("intensificationProbability",
                      &Config::intensificationProbability)
        .def_readonly("circleSectorOverlapTolerance",
                      &Config::circleSectorOverlapTolerance)
        .def_readonly("minCircleSectorSize", &Config::minCircleSectorSize);

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
        .def("pop_sizes", &Statistics::popSizes)
        .def("feasible_pops", &Statistics::feasiblePops)
        .def("pop_diversity", &Statistics::popDiversity)
        .def("curr_objectives", &Statistics::currObjectives)
        .def("best_objectives", &Statistics::bestObjectives);

    py::class_<Result>(m, "Result")
        .def("get_best_found",
             &Result::getBestFound,
             py::return_value_policy::reference)
        .def("get_statistics",
             &Result::getStatistics,
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

    xOps.def("alternating_exchange", &alternatingExchange);
    xOps.def("ordered_exchange", &orderedExchange);
    xOps.def("selective_route_exchange", &selectiveRouteExchange);

    // Local search operators (as a submodule)
    py::module lsOps = m.def_submodule("operators");

    lsOps.def("move_single_client", &moveSingleClient);
    lsOps.def("move_two_clients", &moveTwoClients);
    lsOps.def("move_two_clients_reversed", &moveTwoClientsReversed);
    lsOps.def("swap_two_client_pairs", &swapTwoClientPairs);
    lsOps.def("swap_two_clients_for_one", &swapTwoClientsForOne);
    lsOps.def("swap_two_single_clients", &swapTwoSingleClients);
    lsOps.def("two_opt_between_routes", &twoOptBetweenRoutes);
    lsOps.def("two_opt_within_route", &twoOptWithinRoute);

    lsOps.def("relocate_star", &relocateStar);
    lsOps.def("swap_star", &swapStar);
}
