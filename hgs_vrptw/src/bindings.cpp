#include "Genetic.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Result.h"
#include "Split.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(hgspy, m)
{
    py::class_<LocalSearch>(m, "LocalSearch");

    py::class_<Params>(m, "Params");

    py::class_<Population>(m, "Population");

    py::class_<Split>(m, "Split");

    py::class_<Result>(m, "Result")
        .def(py::init<std::vector<Individual *> const &,
                      std::vector<Individual *> const &>(),
             py::arg("feasible"),
             py::arg("infeasible"))
        .def("get_best_found", &Result::getBestFound)
        .def("export_best_known_solution",
             &Result::writeBestKnowSolution,
             py::arg("path"));

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
    //            [](MasterProblem &master, CutFamily &cutFamily, double tol) {
    //                auto res = master.solveWith(cutFamily, tol);
    //                return carma::col_to_arr(*res);
    //            },
    //            py::arg("cut_family"),
    //            py::arg("tol") = 1e-4)
    //        .def("first_stage_objective", &MasterProblem::firstStageObjective)
    //        .def("second_stage_objective",
    //        &MasterProblem::secondStageObjective) .def("objective",
    //        &MasterProblem::objective);
}
