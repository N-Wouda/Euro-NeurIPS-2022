#include "Config.h"
#include "GeneticAlgorithm.h"
#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Statistics.h"
#include "XorShift128.h"

#include <pybind11/chrono.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(hgspy, m)
{
    py::class_<XorShift128>(m, "XorShift128")
        .def(py::init<int>(), py::arg("seed"));

    py::class_<Individual>(m, "Individual")
        .def("get_routes", &Individual::getRoutes)
        .def("get_tour", &Individual::getTour)
        .def("cost", &Individual::cost);

    py::class_<LocalSearch>(m, "LocalSearch")
        .def(py::init<Params &, XorShift128 &>(),
             py::arg("params"),
             py::arg("rng"));

    py::class_<Config>(m, "Config")
        .def(py::init<int,
                      size_t,
                      int,
                      bool,
                      double,
                      size_t,
                      double,
                      double,
                      double,
                      size_t,
                      size_t,
                      size_t,
                      size_t,
                      size_t,
                      double,
                      size_t,
                      double,
                      int,
                      size_t,
                      int,
                      int,
                      int,
                      bool,
                      bool,
                      int,
                      int>(),
             py::arg("seed") = 0,
             py::arg("nbIter") = 20'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("collectStatistics") = false,
             py::arg("initialTimeWarpPenalty") = 1.,
             py::arg("nbPenaltyManagement") = 100,
             py::arg("penaltyBooster") = 2.,
             py::arg("penaltyIncrease") = 1.2,
             py::arg("penaltyDecrease") = 0.85,
             py::arg("minimumPopulationSize") = 25,
             py::arg("generationSize") = 40,
             py::arg("nbCrossover") = 16,
             py::arg("nbElite") = 4,
             py::arg("nbClose") = 5,
             py::arg("targetFeasible") = 0.2,
             py::arg("repairProbability") = 50,
             py::arg("diversityWeight") = 0.,
             py::arg("nbVeh") = INT_MAX,
             py::arg("nbGranular") = 40,
             py::arg("weightWaitTime") = 2,
             py::arg("weightTimeWarp") = 10,
             py::arg("intensificationProbabilityLS") = 15,
             py::arg("useSwapStarTW") = true,
             py::arg("skipSwapStarDist") = false,
             py::arg("circleSectorOverlapToleranceDegrees") = 0,
             py::arg("minCircleSectorSizeDegrees") = 15)
        .def_readonly("seed", &Config::seed)
        .def_readonly("nbIter", &Config::nbIter)
        .def_readonly("timeLimit", &Config::timeLimit)
        .def_readonly("collectStatistics", &Config::collectStatistics)
        .def_readonly("initialTimeWarpPenalty", &Config::initialTimeWarpPenalty)
        .def_readonly("nbPenaltyManagement", &Config::nbPenaltyManagement)
        .def_readonly("penaltyBooster", &Config::penaltyBooster)
        .def_readonly("penaltyIncrease", &Config::penaltyIncrease)
        .def_readonly("penaltyDecrease", &Config::penaltyDecrease)
        .def_readonly("minimumPopulationSize", &Config::minimumPopulationSize)
        .def_readonly("generationSize", &Config::generationSize)
        .def_readonly("nbCrossover", &Config::nbCrossover)
        .def_readonly("nbElite", &Config::nbElite)
        .def_readonly("nbClose", &Config::nbClose)
        .def_readonly("targetFeasible", &Config::targetFeasible)
        .def_readonly("repairProbability", &Config::repairProbability)
        .def_readonly("diversityWeight", &Config::diversityWeight)
        .def_readonly("nbVeh", &Config::nbVeh)
        .def_readonly("nbGranular", &Config::nbGranular)
        .def_readonly("weightWaitTime", &Config::weightWaitTime)
        .def_readonly("weightTimeWarp", &Config::weightTimeWarp)
        .def_readonly("intensificationProbabilityLS",
                      &Config::intensificationProbabilityLS)
        .def_readonly("useSwapStarTW", &Config::useSwapStarTW)
        .def_readonly("skipSwapStarDist", &Config::skipSwapStarDist)
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
        .def("run_until", &GeneticAlgorithm::runUntil, py::arg("time_point"));
}
