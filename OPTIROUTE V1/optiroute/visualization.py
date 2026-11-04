from __future__ import annotations

from html import escape

from optiroute.warehouse import WarehouseGraph


def render_warehouse_svg(
    graph: WarehouseGraph,
    route: tuple[int, ...],
    start: int,
    destination: int,
    current: int,
    travelled: tuple[int, ...] = (),
    node_roles: dict[int, str] | None = None,
    zones: dict[int, str] | None = None,
    heading: str | None = None,
    animation_path: tuple[int, ...] = (),
    animation_duration_s: float = 0.9,
) -> str:
    spacing = 112
    margin = 52
    width = margin * 2 + spacing * (graph.size - 1)
    height = margin * 2 + spacing * (graph.size - 1) + 32
    blocked = {
        key for key in graph.edge_keys if graph.edge(*key).blocked
    }
    congested = {
        key for key in graph.edge_keys if graph.edge(*key).congestion > 0
    }
    route_edges = {_edge_key(a, b) for a, b in zip(route, route[1:])}
    travelled_edges = {
        _edge_key(a, b) for a, b in zip(travelled, travelled[1:])
    }
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
        'width="100%" height="auto" style="display:block" '
        f'role="img" aria-label="OPTIROUTE warehouse grid and planned route">',
        "<rect width=\"100%\" height=\"100%\" rx=\"18\" fill=\"#0c1420\"/>",
    ]

    for edge in graph.edge_keys:
        left, right = edge
        x1, y1 = _position(graph, left, spacing, margin)
        x2, y2 = _position(graph, right, spacing, margin)
        if edge in blocked:
            color, line_width, dash = "#f87171", 4, ' stroke-dasharray="7 6"'
        elif edge in route_edges:
            color, line_width, dash = "#22d3ee", 8, ""
        elif edge in travelled_edges:
            color, line_width, dash = "#a78bfa", 6, ""
        elif edge in congested:
            color, line_width, dash = "#f59e0b", 5, ""
        else:
            color, line_width, dash = "#334155", 4, ""
        parts.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            f'stroke="{color}" stroke-width="{line_width}" stroke-linecap="round"{dash}/>'
        )
        if edge in blocked:
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            parts.append(
                f'<path d="M {mid_x - 6} {mid_y - 6} l 12 12 m 0 -12 l -12 12" '
                'stroke="#fecaca" stroke-width="3" stroke-linecap="round"/>'
            )

    node_roles = node_roles or {}
    zones = zones or {}
    role_colors = {
        "Pickup": "#c084fc",
        "Delivery": "#f472b6",
        "Restricted": "#64748b",
    }
    zone_colors = {
        "A": "#1d4ed8",
        "B": "#047857",
        "C": "#b45309",
        "D": "#7e22ce",
    }
    for node in range(graph.node_count):
        x, y = _position(graph, node, spacing, margin)
        color = role_colors.get(node_roles.get(node, ""), "#e2e8f0")
        if node == start:
            color = "#34d399"
        if node == destination:
            color = "#60a5fa"
        if node == current and current not in (start, destination):
            color = "#fb923c"
        elif node == current and current == destination:
            color = "#fb923c"
        zone_color = zone_colors.get(zones.get(node, ""))
        if zone_color:
            parts.append(
                f'<circle cx="{x}" cy="{y}" r="29" fill="none" '
                f'stroke="{zone_color}" stroke-width="2" opacity="0.8"/>'
            )
        title = escape(
            f"Node {node}"
            + (f" · {node_roles[node]}" if node in node_roles else "")
            + (f" · Zone {zones[node]}" if node in zones else "")
            + (f" · heading {heading}" if node == current and heading else "")
        )
        parts.append(
            f'<circle cx="{x}" cy="{y}" r="22" fill="{color}" '
            f'stroke="#0c1420" stroke-width="4"><title>{title}</title></circle>'
        )
        parts.append(
            f'<text x="{x}" y="{y + 5}" text-anchor="middle" '
            'font-family="system-ui, sans-serif" font-size="13" '
            f'font-weight="700" fill="#0c1420">{escape(str(node))}</text>'
        )

    if len(animation_path) > 1:
        path_commands = []
        for index, node in enumerate(animation_path):
            x, y = _position(graph, node, spacing, margin)
            path_commands.append(f"{'M' if index == 0 else 'L'} {x} {y}")
        motion_path = " ".join(path_commands)
        parts.append(
            '<g aria-label="Animated simulated robot">'
            '<circle r="15" fill="#fb923c" stroke="#fff7ed" stroke-width="3"/>'
            '<path d="M 0 -9 L 6 6 L 0 3 L -6 6 Z" fill="#431407"/>'
            f'<title>Simulated robot heading {escape(heading or "unknown")}</title>'
            f'<animateMotion dur="{max(animation_duration_s, 0.1):.2f}s" '
            f'path="{motion_path}" rotate="auto" fill="freeze"/>'
            '</g>'
        )

    parts.append("</svg>")
    return "".join(parts)


def _position(
    graph: WarehouseGraph, node: int, spacing: int, margin: int
) -> tuple[int, int]:
    row, column = graph.coordinates(node)
    return margin + column * spacing, margin + row * spacing


def _edge_key(left: int, right: int) -> tuple[int, int]:
    return min(left, right), max(left, right)
