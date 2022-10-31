[![CI](https://github.com/N-Wouda/Euro-NeurIPS-2022/actions/workflows/CI.yml/badge.svg)](https://github.com/N-Wouda/Euro-NeurIPS-2022/actions/workflows/CI.yml)

# EURO Meets NeurIPS 2022 Vehicle Routing Competition

Based on the quickstart code [here](https://github.com/ortec/euro-neurips-vrp-2022-quickstart).

## Solver

Our static solver is based on the hybrid genetic search baseline we received as part of the quickstart code.
We have refactored this solver significantly, making it much more modular and slightly more performant.
We also:
- Introduced a generalised $(N, M)$-Exchange operator
- Added statistics collection
- Separated local search and intensification
- Improved parent selection for crossover by focussing on the diversity of both parents
- Simplified solution state
- Removed many ineffective parameters and constructive heuristics
- Any much more!

Finally, we tuned the remaining parameters in a large-scale numerical experiment.

The dynamic strategy we use (`rollout`) is based on simulating multiple scenarios ahead.
We solve these simulated scenarios quickly using the static solver (in a few hundred milliseconds).
We use the simulation solutions to determine which individuals to postpone in each epoch, and which to dispatch.
After having identified which individuals to dispatch, we solve the resulting epoch instance, again using the static solver.


## How to use

First, one needs to install the required poetry dependencies:
```bash
poetry install
```
Then, one needs to compile the static solver.
Assuming the pybind submodule has been initialised, and `cmake` is available, the following should work:
```bash
cmake -Brelease -Shgs_vrptw
make --directory=release
```
Then, the solver (both static and dynamic) can be called using the `solver.py` script.
It is easiest to run this via the `controller.py` script, as (e.g.):
```bash
python controller.py --instance instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt --epoch_tlim 5 -- python solver.py
```
This runs the solver on the given dynamic instance, with a 5s time limit per epoch.
Solving the static instance is achieved by adding the `--static` flag.
Additional command line options are available, and can be found in the respective scripts.

We also offer several scripts running multiple instances in parallel, which is useful for benchmarking.
These scripts are:
- `analysis.py`, which runs the static solver on all instances, collects statistics, and outputs lots of useful data to a given folder.
- `benchmark.py`, which benchmarks the static solver over all instances.
- `benchmark_dynamic.py`, which benchmarks the dynamic solver over all instances.

Finally, for tuning, we used the `make_dynamic_parameters.py` and `make_static_parameters.py` scripts.
These produce configuration files that can be passed into any of the other scripts mentioned above.
To run the tuning scripts, the optional `tune` dependency group should be installed, using:
```bash
poetry install --only tune
```


## Contributors

### Team
- Doorn, Jasper van <j.m.h.van.doorn@vu.nl>
- Lan, Leon <l.lan@vu.nl>
- Pentinga, Luuk <l.pentinga@rug.nl>
- Wouda, Niels <n.a.wouda@rug.nl>

### Supervision
- Aerts-Veenstra, Marjolein <m.aerts-veenstra@rug.nl>
- Bhulai, Sandjai <s.bhulai@vu.nl>
- Rijal, Arpan <a.rijal@rug.nl>
