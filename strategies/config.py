from __future__ import annotations

import tomli

import tools

hgspy = tools.get_hgspy_module()


class Config:
    def __init__(self, node_ops, route_ops, crossover_ops, static_params):
        self._node_ops = node_ops
        self._route_ops = route_ops
        self._crossover_ops = crossover_ops
        self._static_params = static_params

    @classmethod
    def from_file(cls, where: str) -> Config:
        with open(where, "rb") as fh:
            data = tomli.load(fh)
            return cls(**data)

    def node_ops(self):
        return [getattr(hgspy.operators, op) for op in self._node_ops]

    def route_ops(self):
        return [getattr(hgspy.operators, op) for op in self._route_ops]

    def crossover_ops(self):
        return [getattr(hgspy.crossover, op) for op in self._crossover_ops]

    def static_params(self):
        return self._static_params
