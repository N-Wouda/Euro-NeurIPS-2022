from __future__ import annotations

import tomli

import hgspy


class _SubConfig(dict):
    def node_ops(self):
        return [
            getattr(hgspy.operators, op) for op in self.get("node_ops", [])
        ]

    def route_ops(self):
        return [
            getattr(hgspy.operators, op) for op in self.get("route_ops", [])
        ]

    def crossover_ops(self):
        return [
            getattr(hgspy.crossover, op)
            for op in self.get("crossover_ops", [])
        ]

    def solver_params(self):
        return self.get("params", {})

    def strategy_params(self):
        return self.get("strategy_params", {})

    def strategy(self):
        # There's no obvious default for this string value. If the key is
        # accessed but not set, that should raise an error.
        return self["strategy"]


class Config:
    def __init__(self, **kwargs):
        self._static = _SubConfig(**kwargs.get("static", {}))
        self._dynamic = _SubConfig(**kwargs.get("dynamic", {}))

    @classmethod
    def from_file(cls, where: str) -> Config:
        with open(where, "rb") as fh:
            data = tomli.load(fh)
            return cls(**data)

    def static(self) -> _SubConfig:
        return self._static

    def dynamic(self) -> _SubConfig:
        return self._dynamic
