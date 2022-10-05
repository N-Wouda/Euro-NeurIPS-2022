from functools import partial as _partial

from .random_dispatch import random_dispatch as _random_dispatch

greedy = _partial(_random_dispatch, prob=1)
lazy = _partial(_random_dispatch, prob=0)
random = _partial(_random_dispatch, prob=.5)
