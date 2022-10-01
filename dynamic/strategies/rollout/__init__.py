from types import SimpleNamespace

import yaml

with open("dynamic/strategies/rollout/config.yml") as file:
    config = SimpleNamespace(**yaml.safe_load(file))

from .algorithms import rollout_count
