#include "Config.h"
#include "Genetic.h"
#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
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
                      int,
                      int,
                      double,
                      double,
                      double,
                      double,
                      int,
                      int,
                      int,
                      double,
                      double,
                      size_t,
                      size_t,
                      size_t,
                      size_t,
                      double,
                      size_t,
                      int,
                      int,
                      int,
                      int,
                      int,
                      int,
                      double,
                      int,
                      bool,
                      int,
                      int,
                      bool,
                      bool,
                      int,
                      int,
                      bool>(),
             py::arg("nbIter") = 20'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("seed") = 0,
             py::arg("fractionGeneratedNearest") = 0.05,
             py::arg("fractionGeneratedFurthest") = 0.05,
             py::arg("fractionGeneratedSweep") = 0.05,
             py::arg("fractionGeneratedRandomly") = 0.85,
             py::arg("minSweepFillPercentage") = 60,
             py::arg("maxToleratedCapacityViolation") = 50,
             py::arg("maxToleratedTimeWarp") = 100,
             py::arg("initialTimeWarpPenalty") = 1.,
             py::arg("penaltyBooster") = 2.,
             py::arg("minimumPopulationSize") = 25,
             py::arg("generationSize") = 40,
             py::arg("nbElite") = 4,
             py::arg("nbClose") = 5,
             py::arg("targetFeasible") = 0.2,
             py::arg("repairProbability") = 50,
             py::arg("growNbGranularAfterNonImprovementIterations") = 5'000,
             py::arg("growNbGranularAfterIterations") = 0,
             py::arg("growNbGranularSize") = 0,
             py::arg("growPopulationAfterNonImprovementIterations") = 5'000,
             py::arg("growPopulationAfterIterations") = 0,
             py::arg("growPopulationSize") = 0,
             py::arg("diversityWeight") = 0.,
             py::arg("nbVeh") = INT_MAX,
             py::arg("useDynamicParameters") = false,
             py::arg("nbGranular") = 40,
             py::arg("intensificationProbabilityLS") = 15,
             py::arg("useSwapStarTW") = true,
             py::arg("skipSwapStarDist") = false,
             py::arg("circleSectorOverlapToleranceDegrees") = 0,
             py::arg("minCircleSectorSizeDegrees") = 15,
             py::arg("useSymmetricCorrelatedVertices") = false)
        .def_readonly("nbIter", &Config::nbIter)
        .def_readonly("timeLimit", &Config::timeLimit)
        .def_readonly("seed", &Config::seed)
        .def_readonly("fractionGeneratedNearest",
                      &Config::fractionGeneratedNearest)
        .def_readonly("fractionGeneratedFurthest",
                      &Config::fractionGeneratedFurthest)
        .def_readonly("fractionGeneratedSweep", &Config::fractionGeneratedSweep)
        .def_readonly("fractionGeneratedRandomly",
                      &Config::fractionGeneratedRandomly)
        .def_readonly("minSweepFillPercentage", &Config::minSweepFillPercentage)
        .def_readonly("maxToleratedCapacityViolation",
                      &Config::maxToleratedCapacityViolation)
        .def_readonly("maxToleratedTimeWarp", &Config::maxToleratedTimeWarp)
        .def_readonly("initialTimeWarpPenalty", &Config::initialTimeWarpPenalty)
        .def_readonly("penaltyBooster", &Config::penaltyBooster)
        .def_readonly("minimumPopulationSize", &Config::minimumPopulationSize)
        .def_readonly("generationSize", &Config::generationSize)
        .def_readonly("nbElite", &Config::nbElite)
        .def_readonly("nbClose", &Config::nbClose)
        .def_readonly("targetFeasible", &Config::targetFeasible)
        .def_readonly("repairProbability", &Config::repairProbability)
        .def_readonly("growNbGranularAfterNonImprovementIterations",
                      &Config::growNbGranularAfterNonImprovementIterations)
        .def_readonly("growNbGranularAfterIterations",
                      &Config::growNbGranularAfterIterations)
        .def_readonly("growNbGranularSize", &Config::growNbGranularSize)
        .def_readonly("growPopulationAfterNonImprovementIterations",
                      &Config::growPopulationAfterNonImprovementIterations)
        .def_readonly("growPopulationAfterIterations",
                      &Config::growPopulationAfterIterations)
        .def_readonly("growPopulationSize", &Config::growPopulationSize)
        .def_readonly("diversityWeight", &Config::diversityWeight)
        .def_readonly("nbVeh", &Config::nbVeh)
        .def_readonly("useDynamicParameters", &Config::useDynamicParameters)
        .def_readonly("nbGranular", &Config::nbGranular)
        .def_readonly("intensificationProbabilityLS",
                      &Config::intensificationProbabilityLS)
        .def_readonly("useSwapStarTW", &Config::useSwapStarTW)
        .def_readonly("skipSwapStarDist", &Config::skipSwapStarDist)
        .def_readonly("circleSectorOverlapTolerance",
                      &Config::circleSectorOverlapTolerance)
        .def_readonly("minCircleSectorSize", &Config::minCircleSectorSize)
        .def_readonly("useSymmetricCorrelatedVertices",
                      &Config::useSymmetricCorrelatedVertices);

    py::class_<Params>(m, "Params")
        .def(py::init<Config &,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      int,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      std::vector<std::vector<int>> const &>(),
             py::arg("config"),
             py::arg("coords"),
             py::arg("demands"),
             py::arg("vehicle_cap"),
             py::arg("time_windows"),
             py::arg("service_durations"),
             py::arg("distance_matrix"));

    py::class_<Population>(m, "Population")
        .def(py::init<Params &, XorShift128 &, LocalSearch &>(),
             py::arg("params"),
             py::arg("rng"),
             py::arg("local_search"));

    py::class_<Result>(m, "Result")
        .def("get_best_found", &Result::getBestFound);

    py::class_<Genetic>(m, "Genetic")
        .def(py::init<Params &, XorShift128 &, Population &, LocalSearch &>(),
             py::arg("params"),
             py::arg("rng"),
             py::arg("population"),
             py::arg("local_search"))
        .def("run_until", &Genetic::runUntil, py::arg("time_point"));
}
