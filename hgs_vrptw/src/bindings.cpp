#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Split.h"
#include "Individual.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(hgspy, m)
{
    py::class_<Individual>(m, "Individual")
        .def("export_cvrplib_format",
             &Individual::exportCVRPLibFormat,
             py::arg("path"));

    py::class_<LocalSearch>(m, "LocalSearch")
        .def(py::init<Params *>(), py::arg("params"));

    py::class_<Params>(m, "Params")
        .def(py::init<std::string const &,
                      std::string const &,
                      int,
                      int,
                      int,
                      bool,
                      double,
                      double,
                      double,
                      double,
                      int,
                      int,
                      int,
                      double,
                      double,
                      int,
                      int,
                      int,
                      int,
                      double,
                      int,
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
                      bool,
                      bool>(),
             py::arg("instancePath"),
             py::arg("solPath"),
             py::arg("nbIter") = 20'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("seed") = 0,
             py::arg("useWallClockTime") = false,
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
             py::arg("useSymmetricCorrelatedVertices") = false,
             py::arg("doRepeatUntilTimeLimit") = true);

    py::class_<Population>(m, "Population")
        .def(py::init<Params *, Split *, LocalSearch *>(),
             py::arg("params"),
             py::arg("split"),
             py::arg("local_search"));

    py::class_<Split>(m, "Split").def(py::init<Params *>(), py::arg("params"));

    py::class_<Result>(m, "Result")
        .def(py::init<std::vector<Individual *> const &,
                      std::vector<Individual *> const &>(),
             py::arg("feasible"),
             py::arg("infeasible"))
        .def("get_best_found", &Result::getBestFound);

    py::class_<Genetic>(m, "Genetic")
        .def(py::init<Params *, Split *, Population *, LocalSearch *>(),
             py::arg("params"),
             py::arg("split"),
             py::arg("population"),
             py::arg("local_search"))
        .def("run", &Genetic::run);

    //    py::class_<MasterProblem>(m, "MasterProblem")
    //        .def(py::init<ProblemData &, double, double>(),
    //             py::arg("problem_data"),
    //             py::arg("lower_bound") = 0.,
    //             py::arg("upper_bound") = arma::datum::inf)
    //        .def(
    //            "solve_with",
    //            [](MasterProblem &master, CutFamily &cutFamily, double
    //            tol) {
    //                auto res = master.solveWith(cutFamily, tol);
    //                return carma::col_to_arr(*res);
    //            },
    //            py::arg("cut_family"),
    //            py::arg("tol") = 1e-4)
    //        .def("first_stage_objective",
    //        &MasterProblem::firstStageObjective)
    //        .def("second_stage_objective",
    //        &MasterProblem::secondStageObjective) .def("objective",
    //        &MasterProblem::objective);
}
