from __future__ import annotations

from enum import IntEnum

from optiroute.warehouse import WarehouseGraph


class Heading(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


def route_to_commands(
    graph: WarehouseGraph,
    route: tuple[int, ...],
    initial_heading: Heading = Heading.NORTH,
) -> tuple[str, ...]:
    """Convert adjacent grid nodes into turn-and-move commands."""
    if not route:
        raise ValueError("A route must contain at least one node.")
    for node in route:
        graph.coordinates(node)

    heading = initial_heading
    commands: list[str] = []
    for current, following in zip(route, route[1:]):
        current_row, current_column = graph.coordinates(current)
        next_row, next_column = graph.coordinates(following)
        delta = (next_row - current_row, next_column - current_column)
        direction_by_delta = {
            (-1, 0): Heading.NORTH,
            (0, 1): Heading.EAST,
            (1, 0): Heading.SOUTH,
            (0, -1): Heading.WEST,
        }
        try:
            next_heading = direction_by_delta[delta]
        except KeyError as error:
            raise ValueError(
                f"Route contains non-adjacent nodes {current} and {following}."
            ) from error

        turn = (next_heading - heading) % 4
        command = {0: "FORWARD", 1: "RIGHT", 2: "BACKWARD", 3: "LEFT"}[turn]
        commands.append(command)
        heading = next_heading

    return tuple(commands)
