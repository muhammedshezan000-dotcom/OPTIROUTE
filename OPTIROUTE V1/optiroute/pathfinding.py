from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from time import perf_counter

from optiroute.warehouse import CostWeights, WarehouseGraph


@dataclass(frozen=True, slots=True)
class RouteResult:
    nodes: tuple[int, ...]
    total_cost: float
    expanded_nodes: int
    elapsed_ms: float


def dijkstra(
    graph: WarehouseGraph,
    start: int,
    destination: int,
    weights: CostWeights = CostWeights(),
) -> RouteResult | None:
    return _search(graph, start, destination, weights, use_heuristic=False)


def astar(
    graph: WarehouseGraph,
    start: int,
    destination: int,
    weights: CostWeights = CostWeights(),
) -> RouteResult | None:
    return _search(graph, start, destination, weights, use_heuristic=True)


def breadth_first(
    graph: WarehouseGraph,
    start: int,
    destination: int,
    weights: CostWeights = CostWeights(),
) -> RouteResult | None:
    """Find a route with the fewest corridors, ignoring weights and congestion."""
    from collections import deque

    graph.coordinates(start)
    graph.coordinates(destination)
    weights = weights.normalized()
    started_at = perf_counter()
    queue = deque([start])
    previous: dict[int, int] = {}
    visited = {start}
    expanded = 0

    while queue:
        current = queue.popleft()
        expanded += 1
        if current == destination:
            nodes = _reconstruct_path(previous, start, destination)
            total_cost = sum(
                graph.edge_cost(graph.edge(left, right), weights)
                for left, right in zip(nodes, nodes[1:])
            )
            elapsed_ms = (perf_counter() - started_at) * 1000
            return RouteResult(nodes, total_cost, expanded, elapsed_ms)
        for neighbour, _ in graph.neighbours(
            current, CostWeights(1, 0, 0), destination
        ):
            if neighbour in visited:
                continue
            visited.add(neighbour)
            previous[neighbour] = current
            queue.append(neighbour)

    return None


def _search(
    graph: WarehouseGraph,
    start: int,
    destination: int,
    weights: CostWeights,
    *,
    use_heuristic: bool,
) -> RouteResult | None:
    graph.coordinates(start)
    graph.coordinates(destination)
    weights = weights.normalized()
    started_at = perf_counter()
    start_priority = _heuristic(graph, start, destination, weights) if use_heuristic else 0
    frontier: list[tuple[float, float, int]] = [(start_priority, 0.0, start)]
    distances = {start: 0.0}
    previous: dict[int, int] = {}
    expanded = 0

    while frontier:
        _, current_cost, current = heappop(frontier)
        if current_cost > distances.get(current, float("inf")):
            continue
        expanded += 1
        if current == destination:
            nodes = _reconstruct_path(previous, start, destination)
            elapsed_ms = (perf_counter() - started_at) * 1000
            return RouteResult(nodes, current_cost, expanded, elapsed_ms)

        for neighbour, edge_cost in graph.neighbours(current, weights, destination):
            candidate_cost = current_cost + edge_cost
            if candidate_cost >= distances.get(neighbour, float("inf")):
                continue
            distances[neighbour] = candidate_cost
            previous[neighbour] = current
            priority = candidate_cost
            if use_heuristic:
                priority += _heuristic(graph, neighbour, destination, weights)
            heappush(frontier, (priority, candidate_cost, neighbour))

    return None


def _heuristic(
    graph: WarehouseGraph,
    node: int,
    destination: int,
    weights: CostWeights,
) -> float:
    row, column = graph.coordinates(node)
    destination_row, destination_column = graph.coordinates(destination)
    manhattan_distance = abs(row - destination_row) + abs(column - destination_column)
    return manhattan_distance * graph.minimum_step_cost(weights)


def _reconstruct_path(
    previous: dict[int, int], start: int, destination: int
) -> tuple[int, ...]:
    path = [destination]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return tuple(path)
