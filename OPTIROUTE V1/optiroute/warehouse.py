from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite


@dataclass(frozen=True, slots=True)
class CostWeights:
    distance: float = 0.5
    travel_time: float = 0.3
    congestion: float = 0.2

    def normalized(self) -> CostWeights:
        values = (self.distance, self.travel_time, self.congestion)
        if any(not isfinite(value) or value < 0 for value in values):
            raise ValueError("Cost weights must be finite and non-negative.")
        total = sum(values)
        if total <= 0:
            raise ValueError("At least one cost weight must be greater than zero.")
        return CostWeights(*(value / total for value in values))


@dataclass(frozen=True, slots=True)
class Edge:
    left: int
    right: int
    distance_m: float = 0.2
    speed_limit_mps: float = 0.1
    congestion: float = 0.0
    blocked: bool = False

    @property
    def travel_time_s(self) -> float:
        return self.distance_m / self.speed_limit_mps


class WarehouseGraph:
    """Undirected, four-neighbour grid graph with mutable corridor conditions."""

    def __init__(self, size: int = 5, spacing_m: float = 0.2) -> None:
        if size < 2:
            raise ValueError("Warehouse grid size must be at least 2.")
        if not isfinite(spacing_m) or spacing_m <= 0:
            raise ValueError("Grid spacing must be finite and greater than zero.")
        self.size = size
        self.spacing_m = spacing_m
        self.node_count = size * size
        self._edges: dict[tuple[int, int], Edge] = {}
        self._restricted_nodes: frozenset[int] = frozenset()
        self._build_grid()

    @property
    def edge_keys(self) -> tuple[tuple[int, int], ...]:
        return tuple(self._edges)

    def coordinates(self, node: int) -> tuple[int, int]:
        self._validate_node(node)
        return divmod(node, self.size)

    def edge(self, left: int, right: int) -> Edge:
        key = self._edge_key(left, right)
        try:
            return self._edges[key]
        except KeyError as error:
            raise ValueError(f"Nodes {left} and {right} are not adjacent.") from error

    def set_blocked_edges(self, edges: tuple[tuple[int, int], ...]) -> None:
        blocked_keys = {self._edge_key(left, right) for left, right in edges}
        for key in blocked_keys:
            if key not in self._edges:
                raise ValueError(f"Nodes {key[0]} and {key[1]} are not adjacent.")
        self._edges = {
            key: replace(edge, blocked=key in blocked_keys)
            for key, edge in self._edges.items()
        }

    def set_congestion(self, left: int, right: int, value: float) -> None:
        if not isfinite(value) or not 0 <= value <= 10:
            raise ValueError("Congestion must be between 0 and 10.")
        key = self._edge_key(left, right)
        if key not in self._edges:
            raise ValueError(f"Nodes {left} and {right} are not adjacent.")
        self._edges[key] = replace(self._edges[key], congestion=value)

    def set_edge_properties(
        self, left: int, right: int, distance_m: float, speed_limit_mps: float
    ) -> None:
        if not isfinite(distance_m) or distance_m <= 0:
            raise ValueError("Corridor distance must be finite and greater than zero.")
        if not isfinite(speed_limit_mps) or speed_limit_mps <= 0:
            raise ValueError("Speed limit must be finite and greater than zero.")
        key = self._edge_key(left, right)
        if key not in self._edges:
            raise ValueError(f"Nodes {left} and {right} are not adjacent.")
        self._edges[key] = replace(
            self._edges[key],
            distance_m=distance_m,
            speed_limit_mps=speed_limit_mps,
        )

    def set_restricted_nodes(self, nodes: tuple[int, ...]) -> None:
        for node in nodes:
            self._validate_node(node)
        self._restricted_nodes = frozenset(nodes)

    def neighbours(
        self, node: int, weights: CostWeights, destination: int | None = None
    ) -> tuple[tuple[int, float], ...]:
        self._validate_node(node)
        return tuple(
            (neighbour, self.edge_cost(edge, weights))
            for edge in self._edges.values()
            if not edge.blocked
            and node in (edge.left, edge.right)
            for neighbour in (edge.right if edge.left == node else edge.left,)
            if neighbour not in self._restricted_nodes or neighbour == destination
        )

    def edge_cost(self, edge: Edge, weights: CostWeights) -> float:
        normalized = weights.normalized()
        normalized_distance = edge.distance_m / self.spacing_m
        normalized_time = edge.travel_time_s / 2.0
        normalized_congestion = edge.congestion / 10.0
        return (
            normalized.distance * normalized_distance
            + normalized.travel_time * normalized_time
            + normalized.congestion * normalized_congestion
        )

    def minimum_step_cost(self, weights: CostWeights) -> float:
        costs = (
            self.edge_cost(edge, weights)
            for edge in self._edges.values()
            if not edge.blocked
        )
        return min(costs, default=0.0)

    def _build_grid(self) -> None:
        for row in range(self.size):
            for column in range(self.size):
                node = row * self.size + column
                if column + 1 < self.size:
                    self._add_edge(node, node + 1)
                if row + 1 < self.size:
                    self._add_edge(node, node + self.size)

    def _add_edge(self, left: int, right: int) -> None:
        key = self._edge_key(left, right)
        self._edges[key] = Edge(*key, distance_m=self.spacing_m)

    def _edge_key(self, left: int, right: int) -> tuple[int, int]:
        self._validate_node(left)
        self._validate_node(right)
        return (min(left, right), max(left, right))

    def _validate_node(self, node: int) -> None:
        if not isinstance(node, int) or not 0 <= node < self.node_count:
            raise ValueError(f"Node must be an integer from 0 to {self.node_count - 1}.")
