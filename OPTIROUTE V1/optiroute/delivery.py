from __future__ import annotations

from dataclasses import dataclass
from optiroute.pathfinding import RouteResult, astar, dijkstra
from optiroute.warehouse import CostWeights, WarehouseGraph


@dataclass(frozen=True, slots=True)
class DeliveryPlan:
    stops: tuple[int, ...]
    route: tuple[int, ...]
    total_cost: float


def optimize_delivery_order(
    graph: WarehouseGraph,
    start: int,
    stops: tuple[int, ...],
    weights: CostWeights = CostWeights(),
    algorithm: str = "A*",
) -> DeliveryPlan | None:
    """Find the lowest-cost delivery sequence by evaluating stop permutations."""
    graph.coordinates(start)
    unique_stops = tuple(stop for stop in dict.fromkeys(stops) if stop != start)
    for stop in unique_stops:
        graph.coordinates(stop)
    if len(unique_stops) > 8:
        raise ValueError("Exact delivery sequencing supports at most 8 stops.")
    if not unique_stops:
        return DeliveryPlan((), (start,), 0.0)

    planner = astar if algorithm == "A*" else dijkstra if algorithm == "Dijkstra" else None
    if planner is None:
        raise ValueError("Delivery planner algorithm must be Dijkstra or A*.")

    locations = (start, *unique_stops)
    segments: dict[tuple[int, int], RouteResult | None] = {}
    for source in locations:
        for target in unique_stops:
            if source != target:
                segments[(source, target)] = planner(graph, source, target, weights)

    states: dict[tuple[int, int], tuple[float, tuple[int, ...]]] = {}
    for index, stop in enumerate(unique_stops):
        segment = segments[(start, stop)]
        if segment is not None:
            states[(1 << index, index)] = (segment.total_cost, (index,))

    for mask in range(1, 1 << len(unique_stops)):
        for last in range(len(unique_stops)):
            state = states.get((mask, last))
            if state is None:
                continue
            cost, order = state
            for following, stop in enumerate(unique_stops):
                next_mask = mask | (1 << following)
                if next_mask == mask:
                    continue
                segment = segments[(unique_stops[last], stop)]
                if segment is None:
                    continue
                candidate = (cost + segment.total_cost, (*order, following))
                existing = states.get((next_mask, following))
                if existing is None or candidate[0] < existing[0]:
                    states[(next_mask, following)] = candidate

    full_mask = (1 << len(unique_stops)) - 1
    complete = [
        state
        for (mask, _), state in states.items()
        if mask == full_mask
    ]
    if not complete:
        return None
    total_cost, order_indices = min(complete, key=lambda state: state[0])
    stop_order = tuple(unique_stops[index] for index in order_indices)
    route_nodes = [start]
    current = start
    for stop in stop_order:
        segment = segments[(current, stop)]
        if segment is None:
            return None
        route_nodes.extend(segment.nodes[1:])
        current = stop
    return DeliveryPlan(stop_order, tuple(route_nodes), total_cost)
