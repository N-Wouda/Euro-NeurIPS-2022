from .random import greedy, lazy, random
from .rollout import rollout


STRATEGIES = {
    "greedy": greedy,
    "lazy": lazy,
    "random": random,
    "rollout": rollout,
}
