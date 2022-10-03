import tools
config = tools.load_config("dynamic/strategies/rollout/config.yml")

from .algorithms import rollout_count
