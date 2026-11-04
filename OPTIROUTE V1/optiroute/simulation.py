from __future__ import annotations

from dataclasses import dataclass
from random import Random

from optiroute.pathfinding import RouteResult, astar
from optiroute.warehouse import CostWeights, WarehouseGraph


@dataclass(frozen=True, slots=True)
class SimulationStep:
    position: int
    heading: str | None
    route: RouteResult | None
    moved_distance_m: float
    obstacle: tuple[int, int] | None
    rerouted: bool
    arrived: bool
    blocked: bool


def advance_simulation(
    graph: WarehouseGraph,
    current: int,
    destination: int,
    weights: CostWeights,
    *,
    blocked_edges: tuple[tuple[int, int], ...] = (),
    obstacle_probability: float = 0.0,
    random_seed: int = 0,
) -> SimulationStep:
    """Advance a simulated robot one corridor, optionally adding an obstacle."""
    if not 0 <= obstacle_probability <= 1:
        raise ValueError("Obstacle probability must be between 0 and 1.")
    graph.coordinates(current)
    graph.coordinates(destination)
    graph.set_blocked_edges(blocked_edges)

    route = astar(graph, current, destination, weights)
    if route is None:
        return SimulationStep(current, None, None, 0.0, None, False, False, True)
    if len(route.nodes) == 1:
        return SimulationStep(current, None, route, 0.0, None, False, True, False)

    obstacle: tuple[int, int] | None = None
    rerouted = False
    if Random(random_seed).random() < obstacle_probability:
        obstacle = (route.nodes[0], route.nodes[1])
        blocked = tuple(
            sorted(
                {
                    (min(left, right), max(left, right))
                    for left, right in (*blocked_edges, obstacle)
                }
            )
        )
        graph.set_blocked_edges(blocked)
        alternative = astar(graph, current, destination, weights)
        if alternative is None:
            return SimulationStep(
                current, None, None, 0.0, obstacle, False, False, True
            )
        rerouted = alternative.nodes != route.nodes
        route = alternative

    next_node = route.nodes[1]
    left_row, left_column = graph.coordinates(current)
    right_row, right_column = graph.coordinates(next_node)
    heading = {
        (-1, 0): "NORTH",
        (0, 1): "EAST",
        (1, 0): "SOUTH",
        (0, -1): "WEST",
    }[(right_row - left_row, right_column - left_column)]
    distance = graph.edge(current, next_node).distance_m
    return SimulationStep(
        position=next_node,
        heading=heading,
        route=route,
        moved_distance_m=distance,
        obstacle=obstacle,
        rerouted=rerouted,
        arrived=next_node == destination,
        blocked=False,
    )
