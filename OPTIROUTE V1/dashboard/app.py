"""OPTIROUTE warehouse operations control center."""

from __future__ import annotations

import io
from datetime import datetime
from typing import Any

import pandas as pd
import serial
import streamlit as st
from serial.tools import list_ports

from optiroute.analytics import estimate_congestion
from optiroute.delivery import optimize_delivery_order
from optiroute.pathfinding import RouteResult, astar, breadth_first, dijkstra
from optiroute.storage import (
    load_orders,
    load_routes,
    load_traversals,
    save_order,
    save_route,
    save_traversal,
)
from optiroute.simulation import advance_simulation
from optiroute.visualization import render_warehouse_svg
from optiroute.warehouse import CostWeights, WarehouseGraph
from robot.communication import RobotConnection


PAGES = (
    "Overview",
    "Warehouse Map",
    "Route Optimisation",
    "Warehouse Simulation",
    "Robot Control",
    "Delivery Management",
    "Analytics",
    "System Diagnostics",
)
ALGORITHMS = {
    "A*": astar,
    "Dijkstra": dijkstra,
    "BFS": breadth_first,
}


def _initialize_state() -> None:
    defaults: dict[str, Any] = {
        "start_node": 0,
        "destination": 24,
        "simulation_start": 0,
        "simulation_destination": 24,
        "simulation_running": False,
        "simulation_status": "Ready",
        "simulation_tick": 0,
        "simulation_elapsed_s": 0.0,
        "simulation_distance_m": 0.0,
        "simulation_obstacles": 0,
        "simulation_reroutes": 0,
        "simulation_force_obstacle": False,
        "simulation_events": [],
        "simulation_animation_nodes": [],
        "current_node": 0,
        "heading": "EAST",
        "node_roles": {12: "Pickup", 24: "Delivery"},
        "node_zones": {},
        "blocked_edges": [],
        "corridors": {},
        "active_order_id": None,
        "active_goals": [],
        "goal_index": 0,
        "travelled_nodes": [0],
        "auto_run": False,
        "emergency_stopped": False,
        "predicted_congestion": False,
        "robot_connection": None,
        "robot_connection_error": "",
        "last_robot_message": "Not connected",
        "last_planned_origin": None,
        "last_planned_next": None,
        "obstacles_detected": 0,
        "successful_reroutes": 0,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value.copy() if isinstance(value, (dict, list)) else value
    if "traversed_routes" not in st.session_state:
        st.session_state.traversed_routes = load_traversals()
    if "route_history" not in st.session_state:
        st.session_state.route_history = load_routes()
    if "orders" not in st.session_state:
        orders = load_orders()
        for order in orders:
            if order["status"] == "In progress":
                order["status"] = "Interrupted"
                save_order(order)
        st.session_state.orders = orders


def _edge_id(left: int, right: int) -> str:
    return f"{min(left, right)}:{max(left, right)}"


def _edge_from_id(edge_id: str) -> tuple[int, int]:
    left, right = edge_id.split(":")
    return int(left), int(right)


def _node_label(node: int) -> str:
    row, column = divmod(node, 5)
    role = st.session_state.node_roles.get(node, "")
    suffix = f" · {role}" if role else ""
    return f"Node {node} · ({row}, {column}){suffix}"


def _edge_label(edge_id: str) -> str:
    left, right = _edge_from_id(edge_id)
    return f"{left} ↔ {right}"


def _zone_for_node(node: int) -> str:
    row, column = divmod(node, 5)
    if row < 2:
        return "A" if column < 2 else "B"
    return "C" if column < 2 else "D"


def _make_graph() -> WarehouseGraph:
    graph = WarehouseGraph(size=5)
    graph.set_blocked_edges(
        tuple(_edge_from_id(edge) for edge in st.session_state.blocked_edges)
    )
    graph.set_restricted_nodes(
        tuple(
            node
            for node, role in st.session_state.node_roles.items()
            if role == "Restricted"
        )
    )
    predicted = estimate_congestion(tuple(st.session_state.traversed_routes))
    for edge_id, properties in st.session_state.corridors.items():
        left, right = _edge_from_id(edge_id)
        graph.set_edge_properties(
            left,
            right,
            properties.get("distance_m", graph.spacing_m),
            properties.get("speed_limit_mps", 0.1),
        )
        manual_congestion = properties.get("congestion", 0.0)
        graph.set_congestion(left, right, manual_congestion)
    if st.session_state.predicted_congestion:
        for (left, right), value in predicted.items():
            edge = graph.edge(left, right)
            graph.set_congestion(left, right, max(edge.congestion, value))
    return graph


def _route_metrics(
    graph: WarehouseGraph, nodes: tuple[int, ...]
) -> tuple[float, float, int]:
    distance = 0.0
    travel_time = 0.0
    turns = 0
    previous_direction: tuple[int, int] | None = None
    for left, right in zip(nodes, nodes[1:]):
        edge = graph.edge(left, right)
        distance += edge.distance_m
        travel_time += edge.travel_time_s * (1 + edge.congestion / 10)
        left_row, left_column = graph.coordinates(left)
        right_row, right_column = graph.coordinates(right)
        direction = (right_row - left_row, right_column - left_column)
        if previous_direction is not None and direction != previous_direction:
            turns += 1
        previous_direction = direction
    return distance, travel_time, turns


def _record_route(
    graph: WarehouseGraph,
    route: RouteResult,
    algorithm: str,
    *,
    purpose: str,
) -> None:
    distance, travel_time, turns = _route_metrics(graph, route.nodes)
    shortest_distance = dijkstra(
        graph, route.nodes[0], route.nodes[-1], CostWeights(1, 0, 0)
    )
    shortest_distance_m = (
        _route_metrics(graph, shortest_distance.nodes)[0]
        if shortest_distance is not None
        else 0.0
    )
    st.session_state.route_history.append(
        record := {
            "time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "algorithm": algorithm,
            "purpose": purpose,
            "start": route.nodes[0],
            "destination": route.nodes[-1],
            "nodes": list(route.nodes),
            "distance_m": distance,
            "distance_efficiency": (
                shortest_distance_m / distance * 100 if distance else 100.0
            ),
            "travel_time_s": travel_time,
            "turns": turns,
            "cost": route.total_cost,
            "expanded_nodes": route.expanded_nodes,
            "elapsed_ms": route.elapsed_ms,
            "success": True,
        }
    )
    save_route(record)


def _current_goal() -> int:
    goals = st.session_state.active_goals
    goal_index = st.session_state.goal_index
    if goals and goal_index < len(goals):
        return goals[goal_index]
    return st.session_state.destination


def _active_order() -> dict[str, Any] | None:
    order_id = st.session_state.active_order_id
    if order_id is None:
        return None
    return next(
        (order for order in st.session_state.orders if order["id"] == order_id),
        None,
    )


def _sync_autoplay() -> None:
    st.session_state.auto_run = st.session_state.auto_run_toggle


def _append_alert(message: str) -> None:
    alerts = st.session_state.get("alerts", [])
    alerts.insert(
        0,
        {"time": datetime.now().astimezone().isoformat(timespec="seconds"), "message": message},
    )
    st.session_state.alerts = alerts[:50]


def _advance_robot(graph: WarehouseGraph, weights: CostWeights) -> None:
    if st.session_state.emergency_stopped:
        return
    current = st.session_state.current_node
    goal = _current_goal()
    route = astar(graph, current, goal, weights)
    if route is None:
        st.session_state.obstacles_detected += 1
        _append_alert(f"No safe route from node {current} to node {goal}. Robot stopped.")
        st.session_state.auto_run = False
        return
    if len(route.nodes) < 2:
        goals = st.session_state.active_goals
        if goals and st.session_state.goal_index < len(goals):
            st.session_state.goal_index += 1
            _advance_robot(graph, weights)
        return

    next_node = route.nodes[1]
    previous_node = current
    st.session_state.current_node = next_node
    st.session_state.travelled_nodes.append(next_node)
    st.session_state.traversed_routes.append((previous_node, next_node))
    save_traversal((previous_node, next_node))
    st.session_state.heading = _heading_for(graph, previous_node, next_node)

    goals = st.session_state.active_goals
    if goals and next_node == goals[st.session_state.goal_index]:
        st.session_state.goal_index += 1
        if st.session_state.goal_index >= len(goals):
            order = _active_order()
            if order is not None:
                order["status"] = "Completed"
                order["completed_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
                order["route"] = list(st.session_state.travelled_nodes)
                route_nodes = tuple(st.session_state.travelled_nodes)
                total_cost = sum(
                    graph.edge_cost(graph.edge(left, right), weights)
                    for left, right in zip(route_nodes, route_nodes[1:])
                )
                _record_route(
                    graph,
                    RouteResult(route_nodes, total_cost, len(set(route_nodes)), 0.0),
                    "A*",
                    purpose="Completed delivery",
                )
                save_order(order)
            st.session_state.active_order_id = None
            st.session_state.active_goals = []
            st.session_state.goal_index = 0
            st.session_state.auto_run = False
            _append_alert(f"Delivery {order['id'] if order else ''} completed.")


def _heading_for(graph: WarehouseGraph, left: int, right: int) -> str:
    left_row, left_column = graph.coordinates(left)
    right_row, right_column = graph.coordinates(right)
    return {
        (-1, 0): "NORTH",
        (0, 1): "EAST",
        (1, 0): "SOUTH",
        (0, -1): "WEST",
    }[(right_row - left_row, right_column - left_column)]


def _display_map(
    graph: WarehouseGraph,
    route: tuple[int, ...] = (),
    *,
    start: int | None = None,
    destination: int | None = None,
    animation_path: tuple[int, ...] = (),
    animation_duration_s: float = 0.9,
) -> None:
    svg = render_warehouse_svg(
        graph=graph,
        route=route,
        start=st.session_state.start_node if start is None else start,
        destination=st.session_state.destination if destination is None else destination,
        current=st.session_state.current_node,
        travelled=tuple(st.session_state.travelled_nodes),
        node_roles=st.session_state.node_roles,
        zones={node: st.session_state.node_zones.get(node, _zone_for_node(node)) for node in range(25)},
        heading=st.session_state.heading,
        animation_path=animation_path,
        animation_duration_s=animation_duration_s,
    )
    st.html(svg)
    st.caption(
        "Green: start · Blue: selected destination · Orange: digital twin · "
        "Cyan: route · Red: blocked corridor · Amber: congestion · "
        "Purple ring: warehouse zone"
    )


def _show_route_metrics(graph: WarehouseGraph, route: RouteResult) -> None:
    distance, travel_time, turns = _route_metrics(graph, route.nodes)
    columns = st.columns(4)
    columns[0].metric("Weighted cost", f"{route.total_cost:.2f}")
    columns[1].metric("Distance", f"{distance:.2f} m")
    columns[2].metric("Estimated time", f"{travel_time:.1f} s")
    columns[3].metric("Turns", turns)


def _overview(graph: WarehouseGraph, weights: CostWeights) -> None:
    st.subheader("Operations overview")
    pending = sum(order["status"] == "Pending" for order in st.session_state.orders)
    completed = sum(order["status"] == "Completed" for order in st.session_state.orders)
    status = "Emergency stop" if st.session_state.emergency_stopped else (
        "Delivery active" if st.session_state.active_order_id else "Simulation ready"
    )
    metric_columns = st.columns(5)
    metric_columns[0].metric("Warehouse nodes", graph.node_count)
    metric_columns[1].metric("Robot status", status, f"Node {st.session_state.current_node}")
    metric_columns[2].metric("Pending orders", pending)
    metric_columns[3].metric("Deliveries completed", completed)
    metric_columns[4].metric("Blocked corridors", len(st.session_state.blocked_edges))
    target = _current_goal()
    route = astar(graph, st.session_state.current_node, target, weights)
    map_column, details_column = st.columns([1.5, 1])
    with map_column:
        st.subheader("Warehouse digital twin")
        _display_map(graph, route.nodes if route else (), destination=target)
    with details_column:
        st.subheader("System status")
        connection = st.session_state.robot_connection
        connected = connection is not None and connection.is_open
        st.write(f"**Controller:** {'Connected' if connected else 'Simulation only'}")
        st.write(f"**Current heading:** {st.session_state.heading.title()}")
        st.write(f"**Current goal:** Node {target}")
        if route is None:
            st.error("No safe route to the current goal.")
        else:
            _show_route_metrics(graph, route)
        if st.session_state.get("alerts"):
            st.subheader("Latest alerts")
            for alert in st.session_state.alerts[:3]:
                st.warning(f"{alert['time']} — {alert['message']}")


def _warehouse_map(graph: WarehouseGraph) -> None:
    st.subheader("Interactive warehouse editor")
    st.caption(
        "Use the controls to edit nodes and corridors. Changes are applied immediately "
        "to route planning and the digital twin."
    )
    _display_map(graph)
    edge_ids = [_edge_id(*edge) for edge in graph.edge_keys]
    with st.expander("Edit a node or warehouse zone", expanded=True):
        node_column, role_column, zone_column = st.columns(3)
        with node_column:
            node = st.selectbox("Node", range(25), format_func=_node_label, key="map_node")
        with role_column:
            role = st.selectbox(
                "Location type",
                ("Intersection", "Pickup", "Delivery", "Restricted", "Clear role"),
                key="map_role",
            )
        with zone_column:
            zone = st.selectbox("Warehouse zone", ("A", "B", "C", "D"), key="map_zone")
        if st.button("Apply node and zone", type="primary"):
            if role == "Clear role":
                st.session_state.node_roles.pop(node, None)
            elif role == "Intersection":
                st.session_state.node_roles.pop(node, None)
            else:
                st.session_state.node_roles[node] = role
            st.session_state.node_zones[node] = zone
            st.rerun()

    with st.expander("Manage corridors", expanded=True):
        blocked = st.multiselect(
            "Blocked corridors",
            edge_ids,
            default=st.session_state.blocked_edges,
            format_func=_edge_label,
            key="map_blocked",
        )
        st.session_state.blocked_edges = blocked
        corridor_id = st.selectbox(
            "Corridor to configure", edge_ids, format_func=_edge_label
        )
        properties = st.session_state.corridors.get(corridor_id, {})
        edge_left, edge_right = _edge_from_id(corridor_id)
        edge = graph.edge(edge_left, edge_right)
        distance = st.number_input(
            "Corridor distance (m)",
            min_value=0.05,
            max_value=5.0,
            value=float(properties.get("distance_m", edge.distance_m)),
            step=0.05,
        )
        speed = st.number_input(
            "Speed limit (m/s)",
            min_value=0.02,
            max_value=2.0,
            value=float(properties.get("speed_limit_mps", edge.speed_limit_mps)),
            step=0.02,
        )
        congestion_value = st.slider(
            "Manual congestion (0–10)",
            0.0,
            10.0,
            float(properties.get("congestion", edge.congestion)),
            0.5,
        )
        if st.button("Save corridor properties"):
            st.session_state.corridors[corridor_id] = {
                "distance_m": distance,
                "speed_limit_mps": speed,
                "congestion": congestion_value,
            }
            st.rerun()
    st.toggle(
        "Apply historical congestion estimates",
        key="predicted_congestion",
        help="Uses observed simulated traversals; this is a statistical baseline, not machine learning.",
    )


def _route_optimisation(graph: WarehouseGraph, weights: CostWeights) -> None:
    st.subheader("Route optimisation")
    left, right = st.columns(2)
    with left:
        start = st.selectbox(
            "Route start",
            range(25),
            index=st.session_state.start_node,
            format_func=_node_label,
            key="planner_start",
        )
    with right:
        destination = st.selectbox(
            "Route destination",
            range(25),
            index=st.session_state.destination,
            format_func=_node_label,
            key="planner_destination",
        )
    algorithm = st.selectbox("Primary algorithm", tuple(ALGORITHMS), index=0)
    comparison: list[dict[str, Any]] = []
    comparison_results: dict[str, RouteResult | None] = {}
    for name, planner in ALGORITHMS.items():
        result = planner(graph, start, destination, weights)
        comparison_results[name] = result
        comparison.append(
            {
                "Algorithm": name,
                "Route cost": result.total_cost if result else None,
                "Corridors": len(result.nodes) - 1 if result else None,
                "Expanded nodes": result.expanded_nodes if result else None,
                "Search time (ms)": result.elapsed_ms if result else None,
            }
        )
    route = comparison_results[algorithm]
    st.session_state.start_node = start
    st.session_state.destination = destination
    if route is None:
        st.error("No route exists with the current blocked and restricted locations.")
    else:
        st.code(" → ".join(map(str, route.nodes)))
        _show_route_metrics(graph, route)
        _display_map(graph, route.nodes, start=start, destination=destination)
        if st.button("Save route to history", type="primary"):
            for name, result in comparison_results.items():
                if result is not None:
                    _record_route(
                        graph,
                        result,
                        name,
                        purpose="Algorithm comparison",
                    )
            st.success("Dijkstra, A*, and BFS results saved to route history.")

    st.subheader("Compare pathfinding algorithms")
    st.dataframe(comparison, hide_index=True, width="stretch")
    st.caption("BFS minimizes corridor count and intentionally ignores speed and congestion.")

    st.subheader("Multi-delivery route sequencing")
    delivery_nodes = [
        node for node, role in st.session_state.node_roles.items() if role == "Delivery"
    ]
    candidates = delivery_nodes or [24]
    stops = st.multiselect(
        "Delivery stops (exact sequencing supports up to 8)",
        candidates,
        default=candidates[: min(3, len(candidates))],
        format_func=_node_label,
    )
    if stops:
        plan = optimize_delivery_order(graph, start, tuple(stops), weights, algorithm)
        if plan is None:
            st.error("At least one delivery stop is unreachable.")
        else:
            st.write(f"**Optimized stop order:** {' → '.join(map(str, plan.stops))}")
            st.write(f"**Combined route:** {' → '.join(map(str, plan.route))}")
            st.metric("Combined weighted cost", f"{plan.total_cost:.2f}")
            if st.button("Save multi-delivery route"):
                legs = tuple(plan.route)
                distance, travel_time, turns = _route_metrics(graph, legs)
                route_result = RouteResult(
                    legs,
                    plan.total_cost,
                    len(set(legs)),
                    0.0,
                )
                _record_route(graph, route_result, algorithm, purpose="Multi-delivery")
                st.success(
                    f"Saved route · {distance:.1f} m · {travel_time:.1f} s · {turns} turns."
                )


@st.fragment(run_every=1)
def _robot_twin(graph: WarehouseGraph, weights: CostWeights) -> None:
    initial_goal = _current_goal()
    initial_route = astar(
        graph, st.session_state.current_node, initial_goal, weights
    )
    move_column, autoplay_column, reset_column, stop_column = st.columns(4)
    with move_column:
        step_clicked = st.button(
            "Advance one junction",
            type="primary",
            disabled=(
                st.session_state.emergency_stopped
                or initial_route is None
                or len(initial_route.nodes) < 2
            ),
            width="stretch",
            key="robot_step",
        )
    with autoplay_column:
        st.session_state.auto_run_toggle = st.session_state.auto_run
        st.toggle(
            "Animate (1 step/sec)",
            key="auto_run_toggle",
            on_change=_sync_autoplay,
        )
    with reset_column:
        if st.button("Reset position", width="stretch"):
            st.session_state.current_node = st.session_state.start_node
            st.session_state.travelled_nodes = [st.session_state.current_node]
            st.session_state.heading = "EAST"
            st.session_state.auto_run = False
            st.session_state.emergency_stopped = False
    with stop_column:
        if st.button("Emergency stop", width="stretch"):
            st.session_state.auto_run = False
            st.session_state.emergency_stopped = True
            _append_alert("Simulation emergency stop activated.")

    if step_clicked or (
        st.session_state.auto_run
        and initial_route is not None
        and len(initial_route.nodes) > 1
    ):
        _advance_robot(graph, weights)
    elif st.session_state.auto_run:
        st.session_state.auto_run = False
    goal = _current_goal()
    current = st.session_state.current_node
    route = astar(graph, current, goal, weights)
    if route is not None and len(route.nodes) > 1:
        next_node = route.nodes[1]
        previous_origin = st.session_state.last_planned_origin
        previous_next = st.session_state.last_planned_next
        if (
            previous_origin == current
            and previous_next is not None
            and previous_next != current
            and previous_next != next_node
        ):
            st.session_state.successful_reroutes += 1
            _append_alert(
                f"Route recalculated at node {current}: corridor conditions changed."
            )
        st.session_state.last_planned_origin = current
        st.session_state.last_planned_next = next_node
    columns = st.columns(4)
    columns[0].metric("Digital-twin position", f"Node {current}")
    columns[1].metric("Heading", st.session_state.heading.title())
    columns[2].metric("Mission goal", f"Node {goal}")
    columns[3].metric(
        "Mission status",
        "Emergency stopped" if st.session_state.emergency_stopped else "Ready",
    )
    if route is None:
        st.error("No safe route to the current goal. The robot remains stopped.")
    else:
        _display_map(graph, route.nodes, destination=goal)
    st.subheader("Recent simulated movement")
    st.write(" → ".join(str(node) for node in st.session_state.travelled_nodes[-30:]))
    if st.session_state.active_order_id:
        st.info(
            f"Executing delivery {st.session_state.active_order_id}: "
            f"stop {st.session_state.goal_index + 1} of {len(st.session_state.active_goals)}."
        )


def _robot_control(graph: WarehouseGraph, weights: CostWeights) -> None:
    st.subheader("Robot control and digital twin")
    st.warning(
        "This page controls the software simulation. Physical motion commands remain disabled "
        "until the actual robot, sensors, motor driver, and emergency stop are calibrated."
    )
    _robot_twin(graph, weights)
    if not st.session_state.emergency_stopped:
        route = astar(graph, st.session_state.current_node, _current_goal(), weights)
        if route is not None and len(route.nodes) > 1:
            if st.button("Simulate obstacle on next corridor"):
                obstacle = _edge_id(route.nodes[0], route.nodes[1])
                if obstacle not in st.session_state.blocked_edges:
                    st.session_state.blocked_edges.append(obstacle)
                    st.session_state.obstacles_detected += 1
                    _append_alert(
                        f"Simulated obstacle detected on corridor {_edge_label(obstacle)}."
                    )
                st.rerun()
    st.caption("Obstacle and corridor changes immediately affect the next route calculation.")


def _start_simulation(start: int, destination: int) -> None:
    st.session_state.simulation_start = start
    st.session_state.simulation_destination = destination
    st.session_state.start_node = start
    st.session_state.destination = destination
    st.session_state.current_node = start
    st.session_state.travelled_nodes = [start]
    st.session_state.heading = "EAST"
    st.session_state.simulation_running = True
    st.session_state.simulation_status = "Running"
    st.session_state.simulation_tick = 0
    st.session_state.simulation_elapsed_s = 0.0
    st.session_state.simulation_distance_m = 0.0
    st.session_state.simulation_obstacles = 0
    st.session_state.simulation_reroutes = 0
    st.session_state.simulation_force_obstacle = False
    st.session_state.simulation_events = []
    st.session_state.simulation_animation_nodes = []
    st.session_state.emergency_stopped = False
    st.session_state.active_goals = []
    st.session_state.active_order_id = None


def _reset_simulation(clear_obstacles: bool = False) -> None:
    st.session_state.simulation_running = False
    st.session_state.simulation_status = "Ready"
    st.session_state.current_node = st.session_state.simulation_start
    st.session_state.start_node = st.session_state.simulation_start
    st.session_state.destination = st.session_state.simulation_destination
    st.session_state.travelled_nodes = [st.session_state.current_node]
    st.session_state.heading = "EAST"
    st.session_state.simulation_tick = 0
    st.session_state.simulation_elapsed_s = 0.0
    st.session_state.simulation_distance_m = 0.0
    st.session_state.simulation_obstacles = 0
    st.session_state.simulation_reroutes = 0
    st.session_state.simulation_force_obstacle = False
    st.session_state.simulation_events = []
    st.session_state.simulation_animation_nodes = []
    st.session_state.emergency_stopped = False
    if clear_obstacles:
        st.session_state.blocked_edges = []


def _simulation_tick(
    graph: WarehouseGraph,
    weights: CostWeights,
    obstacle_probability: float,
) -> None:
    current = st.session_state.current_node
    destination = st.session_state.simulation_destination
    blocked_edges = tuple(
        _edge_from_id(edge_id) for edge_id in st.session_state.blocked_edges
    )
    probability = (
        1.0
        if st.session_state.simulation_force_obstacle
        else obstacle_probability
    )
    step = advance_simulation(
        graph,
        current,
        destination,
        weights,
        blocked_edges=blocked_edges,
        obstacle_probability=probability,
        random_seed=42 + st.session_state.simulation_tick,
    )
    st.session_state.simulation_tick += 1
    st.session_state.simulation_force_obstacle = False

    if step.obstacle is not None:
        edge_id = _edge_id(*step.obstacle)
        if edge_id not in st.session_state.blocked_edges:
            st.session_state.blocked_edges.append(edge_id)
        st.session_state.simulation_obstacles += 1
        st.session_state.obstacles_detected += 1
        message = f"Obstacle injected on corridor {_edge_label(edge_id)}."
        if step.rerouted:
            st.session_state.simulation_reroutes += 1
            st.session_state.successful_reroutes += 1
            message += " A* found an alternative route."
        _append_alert(message)
        st.session_state.simulation_events.insert(
            0, {"tick": st.session_state.simulation_tick, "event": message}
        )

    if step.blocked:
        st.session_state.simulation_status = "Blocked"
        st.session_state.simulation_running = False
        message = f"No safe route from node {current} to {destination}; simulation paused."
        st.session_state.simulation_events.insert(
            0, {"tick": st.session_state.simulation_tick, "event": message}
        )
        _append_alert(message)
        return

    if step.position != current:
        st.session_state.current_node = step.position
        st.session_state.travelled_nodes.append(step.position)
        st.session_state.simulation_animation_nodes.append(step.position)
        st.session_state.traversed_routes.append((current, step.position))
        save_traversal((current, step.position))
        st.session_state.heading = step.heading or st.session_state.heading
        edge = graph.edge(current, step.position)
        st.session_state.simulation_distance_m += edge.distance_m
        st.session_state.simulation_elapsed_s += (
            edge.travel_time_s * (1 + edge.congestion / 10)
        )
    if step.arrived:
        st.session_state.simulation_status = "Arrived"
        st.session_state.simulation_running = False
        route_nodes = tuple(st.session_state.travelled_nodes)
        route_cost = sum(
            graph.edge_cost(graph.edge(left, right), weights)
            for left, right in zip(route_nodes, route_nodes[1:])
        )
        _record_route(
            graph,
            RouteResult(route_nodes, route_cost, len(set(route_nodes)), 0.0),
            "A*",
            purpose="Warehouse simulation",
        )
        message = f"Simulation delivery reached node {destination}."
        st.session_state.simulation_events.insert(
            0, {"tick": st.session_state.simulation_tick, "event": message}
        )
        _append_alert(message)


@st.fragment(run_every=1)
def _simulation_live_panel(
    graph: WarehouseGraph,
    weights: CostWeights,
    obstacle_probability: float,
    steps_per_second: int,
) -> None:
    if st.session_state.simulation_running:
        st.session_state.simulation_animation_nodes = [
            st.session_state.current_node
        ]
        for _ in range(steps_per_second):
            _simulation_tick(graph, weights, obstacle_probability)
            if not st.session_state.simulation_running:
                break
    elif not st.session_state.simulation_animation_nodes:
        st.session_state.simulation_animation_nodes = [
            st.session_state.current_node
        ]

    destination = st.session_state.simulation_destination
    route = astar(graph, st.session_state.current_node, destination, weights)
    if route is None:
        st.error("No route remains. Clear a blockage or reset the simulation.")
    else:
        animation_path = tuple(st.session_state.simulation_animation_nodes)
        if len(animation_path) < 2:
            animation_path = (st.session_state.current_node,)
        _display_map(
            graph,
            route.nodes,
            start=st.session_state.simulation_start,
            destination=destination,
            animation_path=animation_path,
            animation_duration_s=0.9,
        )
        st.session_state.simulation_animation_nodes = []
    metrics = st.columns(6)
    metrics[0].metric("Simulation", st.session_state.simulation_status)
    metrics[1].metric("Robot node", st.session_state.current_node)
    metrics[2].metric("Heading", st.session_state.heading.title())
    metrics[3].metric(
        "Simulated distance", f"{st.session_state.simulation_distance_m:.2f} m"
    )
    metrics[4].metric(
        "Simulated travel time", f"{st.session_state.simulation_elapsed_s:.1f} s"
    )
    metrics[5].metric(
        "Obstacles / reroutes",
        f"{st.session_state.simulation_obstacles} / {st.session_state.simulation_reroutes}",
    )

    controls = st.columns(4)
    with controls[0]:
        if st.button(
            "Pause simulation",
            disabled=not st.session_state.simulation_running,
            width="stretch",
        ):
            st.session_state.simulation_running = False
            st.session_state.simulation_status = "Paused"
    with controls[1]:
        if st.button(
            "Resume simulation",
            disabled=st.session_state.simulation_running
            or st.session_state.simulation_status in ("Arrived", "Blocked"),
            width="stretch",
        ):
            st.session_state.simulation_running = True
            st.session_state.simulation_status = "Running"
    with controls[2]:
        if st.button(
            "Inject obstacle",
            disabled=st.session_state.simulation_status
            in ("Ready", "Arrived", "Blocked"),
            width="stretch",
        ):
            st.session_state.simulation_force_obstacle = True
    with controls[3]:
        if st.button("Emergency stop", width="stretch"):
            st.session_state.simulation_running = False
            st.session_state.simulation_status = "Emergency stopped"
            st.session_state.emergency_stopped = True
            _append_alert("Warehouse simulation emergency stop activated.")

    if st.session_state.simulation_events:
        st.subheader("Simulation event log")
        st.dataframe(
            st.session_state.simulation_events[:20],
            hide_index=True,
            width="stretch",
        )


def _warehouse_simulation(graph: WarehouseGraph, weights: CostWeights) -> None:
    st.subheader("Discrete-time warehouse digital twin")
    st.caption(
        "Run a simulated delivery through the warehouse. Each tick moves one junction; "
        "obstacles can be injected manually or probabilistically, then A* replans."
    )
    start_column, destination_column = st.columns(2)
    with start_column:
        start = st.selectbox(
            "Simulation start",
            range(graph.node_count),
            index=st.session_state.simulation_start,
            format_func=_node_label,
            key="simulation_start_selector",
        )
    with destination_column:
        destination = st.selectbox(
            "Simulation destination",
            range(graph.node_count),
            index=st.session_state.simulation_destination,
            format_func=_node_label,
            key="simulation_destination_selector",
        )
    settings = st.columns(3)
    with settings[0]:
        speed = st.selectbox(
            "Simulation speed",
            (1, 2, 4),
            format_func=lambda value: f"{value} junctions / second",
        )
    with settings[1]:
        obstacle_rate = st.slider(
            "Random obstacle chance per tick",
            0.0,
            0.5,
            0.0,
            0.05,
            help="Deterministic per simulation tick so a run can be repeated.",
        )
    with settings[2]:
        clear_obstacles = st.checkbox("Clear obstacles when resetting", value=True)

    actions = st.columns(3)
    with actions[0]:
        if st.button("Start new simulation", type="primary", width="stretch"):
            _start_simulation(start, destination)
    with actions[1]:
        if st.button("Reset simulation", width="stretch"):
            _reset_simulation(clear_obstacles=clear_obstacles)
    with actions[2]:
        st.metric("Simulation ticks", st.session_state.simulation_tick)

    if start == destination:
        st.info("Choose different start and destination nodes to simulate a delivery.")
        return
    if st.session_state.simulation_status == "Ready":
        st.info("Select a scenario and choose **Start new simulation**.")
        return
    _simulation_live_panel(graph, weights, obstacle_rate, speed)


def _delivery_management(graph: WarehouseGraph, weights: CostWeights) -> None:
    st.subheader("Delivery management")
    pickup_nodes = [
        node for node, role in st.session_state.node_roles.items() if role == "Pickup"
    ]
    delivery_nodes = [
        node for node, role in st.session_state.node_roles.items() if role == "Delivery"
    ]
    with st.form("create_delivery"):
        order_name = st.text_input("Order reference", value=f"ORD-{len(st.session_state.orders) + 1:03d}")
        pickup = st.selectbox(
            "Pickup location",
            pickup_nodes or [12],
            format_func=_node_label,
        )
        destinations = st.multiselect(
            "Delivery stations",
            delivery_nodes or [24],
            default=delivery_nodes[:1] or [24],
            format_func=_node_label,
        )
        submitted = st.form_submit_button("Add delivery order", type="primary")
    if submitted:
        if not order_name.strip():
            st.error("Enter an order reference.")
        elif not destinations:
            st.error("Select at least one delivery station.")
        elif any(order["id"] == order_name.strip() for order in st.session_state.orders):
            st.error("That order reference already exists.")
        else:
            st.session_state.orders.append(
                new_order := {
                    "id": order_name.strip(),
                    "pickup": pickup,
                    "destinations": destinations,
                    "status": "Pending",
                    "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                    "completed_at": "",
                    "route": [],
                }
            )
            save_order(new_order)
            st.success(f"Order {order_name.strip()} added.")

    if not st.session_state.orders:
        st.info("No delivery orders yet. Add pickup and delivery nodes on the warehouse map.")
        return
    st.dataframe(
        [
            {
                "Order": order["id"],
                "Pickup": order["pickup"],
                "Stops": ", ".join(map(str, order["destinations"])),
                "Status": order["status"],
                "Created": order["created_at"],
            }
            for order in st.session_state.orders
        ],
        hide_index=True,
        width="stretch",
    )
    pending_orders = [order for order in st.session_state.orders if order["status"] == "Pending"]
    if pending_orders and st.session_state.active_order_id is None:
        selected_order = st.selectbox(
            "Dispatch order",
            pending_orders,
            format_func=lambda order: f"{order['id']} · pickup {order['pickup']}",
        )
        plan = optimize_delivery_order(
            graph,
            selected_order["pickup"],
            tuple(selected_order["destinations"]),
            weights,
        )
        if plan is None:
            st.error("This delivery order contains unreachable destinations.")
        else:
            pickup_route = astar(
                graph, st.session_state.current_node, selected_order["pickup"], weights
            )
            if pickup_route is None:
                st.error("The pickup location is unreachable from the robot.")
                return
            full_route = pickup_route.nodes + plan.route[1:]
            st.write(
                f"Pickup first, then optimized stops: "
                f"{selected_order['pickup']} → {' → '.join(map(str, plan.stops))}"
            )
            st.write(f"Planned route: {' → '.join(map(str, full_route))}")
            if st.button("Dispatch to simulated robot", type="primary"):
                selected_order["status"] = "In progress"
                selected_order["dispatched_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
                save_order(selected_order)
                st.session_state.active_order_id = selected_order["id"]
                st.session_state.active_goals = [
                    selected_order["pickup"],
                    *plan.stops,
                ]
                st.session_state.goal_index = 0
                st.session_state.destination = plan.stops[0]
                st.session_state.travelled_nodes = [st.session_state.current_node]
                st.session_state.last_planned_origin = None
                st.session_state.last_planned_next = None
                st.session_state.emergency_stopped = False
                st.rerun()
    elif st.session_state.active_order_id:
        st.info(f"Active order: {st.session_state.active_order_id}. Control it on the Robot Control page.")

    interrupted_orders = [
        order for order in st.session_state.orders if order["status"] == "Interrupted"
    ]
    if interrupted_orders:
        st.warning(
            "An interrupted order needs a position check before it can be dispatched again."
        )
        interrupted = st.selectbox(
            "Interrupted order",
            interrupted_orders,
            format_func=lambda order: order["id"],
        )
        if st.button("Requeue after confirming robot location"):
            interrupted["status"] = "Pending"
            interrupted.pop("dispatched_at", None)
            save_order(interrupted)
            st.success("Order returned to the pending queue.")
            st.rerun()

    completed = [order for order in st.session_state.orders if order["status"] == "Completed"]
    if completed:
        st.subheader("Completed deliveries")
        st.dataframe(
            [
                {
                    "Order": order["id"],
                    "Completed": order.get("completed_at", ""),
                    "Route": " → ".join(map(str, order.get("route", []))),
                }
                for order in completed
            ],
            hide_index=True,
            width="stretch",
        )


def _analytics(graph: WarehouseGraph) -> None:
    st.subheader("Performance analytics")
    history = st.session_state.route_history
    completed = [order for order in st.session_state.orders if order["status"] == "Completed"]
    attempted_orders = sum(bool(order.get("dispatched_at")) for order in st.session_state.orders)
    traversed_routes = st.session_state.traversed_routes
    average_time = (
        sum(record["travel_time_s"] for record in history) / len(history)
        if history
        else 0
    )
    delivery_durations = []
    for order in completed:
        dispatched_at = order.get("dispatched_at")
        completed_at = order.get("completed_at")
        if dispatched_at and completed_at:
            delivery_durations.append(
                (
                    datetime.fromisoformat(completed_at)
                    - datetime.fromisoformat(dispatched_at)
                ).total_seconds()
            )
    average_delivery_time = (
        sum(delivery_durations) / len(delivery_durations)
        if delivery_durations
        else 0
    )
    average_efficiency = (
        sum(record["distance_efficiency"] for record in history) / len(history)
        if history
        else 0
    )
    travelled_distance = sum(
        graph.edge(a, b).distance_m
        for route in traversed_routes
        for a, b in zip(route, route[1:])
    )
    metrics = st.columns(7)
    metrics[0].metric("Completed deliveries", len(completed))
    metrics[1].metric(
        "Navigation success rate",
        f"{len(completed) / attempted_orders * 100:.1f}%" if attempted_orders else "—",
    )
    metrics[2].metric("Planned route records", len(history))
    metrics[3].metric("Travelled distance", f"{travelled_distance:.1f} m")
    metrics[4].metric("Average delivery time", f"{average_delivery_time:.1f} s")
    metrics[5].metric("Average distance efficiency", f"{average_efficiency:.1f}%")
    metrics[6].metric(
        "Obstacle / reroute events",
        f"{st.session_state.obstacles_detected} / {st.session_state.successful_reroutes}",
    )
    st.caption(f"Average planned route estimate: {average_time:.1f} seconds.")
    if not history:
        st.info("Plan and save routes to populate algorithm and performance charts.")
        return

    data = pd.DataFrame(history)
    left, right = st.columns(2)
    with left:
        st.subheader("Algorithm search time")
        st.bar_chart(data, x="algorithm", y="elapsed_ms", color="algorithm")
        st.subheader("Distance versus estimated time")
        st.scatter_chart(data, x="distance_m", y="travel_time_s", color="algorithm")
    with right:
        st.subheader("Route cost history")
        st.line_chart(data, x="time", y="cost", color="algorithm")
        st.subheader("Recorded route length")
        st.bar_chart(data, x="time", y="distance_m", color="purpose")
    alternatives = data[data["purpose"] == "Algorithm comparison"]
    if not alternatives.empty:
        st.subheader("Original route versus algorithm alternatives")
        st.bar_chart(alternatives, x="algorithm", y="cost", color="algorithm")

    completed_data = pd.DataFrame(completed)
    if not completed_data.empty:
        completed_data["day"] = pd.to_datetime(
            completed_data["completed_at"]
        ).dt.date.astype(str)
        daily_deliveries = (
            completed_data.groupby("day", as_index=False)
            .size()
            .rename(columns={"size": "deliveries"})
        )
        st.subheader("Daily delivery performance")
        st.bar_chart(daily_deliveries, x="day", y="deliveries")

    usage = estimate_congestion(tuple(traversed_routes))
    if usage:
        congestion_data = pd.DataFrame(
            [
                {"Corridor": _edge_label(_edge_id(*edge)), "Estimated congestion": value}
                for edge, value in sorted(usage.items())
            ]
        )
        st.subheader("Observed corridor traffic")
        st.bar_chart(congestion_data, x="Corridor", y="Estimated congestion")

    with st.expander("Route history"):
        st.dataframe(
            data[
                [
                    "time",
                    "purpose",
                    "algorithm",
                    "start",
                    "destination",
                    "distance_m",
                    "distance_efficiency",
                    "travel_time_s",
                    "turns",
                    "cost",
                    "elapsed_ms",
                ]
            ],
            hide_index=True,
            width="stretch",
        )
        csv_buffer = io.StringIO()
        data.to_csv(csv_buffer, index=False)
        st.download_button(
            "Download route history CSV",
            csv_buffer.getvalue(),
            file_name="optiroute-route-history.csv",
            mime="text/csv",
        )


def _diagnostics() -> None:
    st.subheader("System diagnostics")
    graph = _make_graph()
    rows = [
        {"Check": "Warehouse graph", "Status": f"{graph.node_count} nodes, {len(graph.edge_keys)} corridors"},
        {"Check": "Pathfinding", "Status": "Dijkstra, A*, BFS available"},
        {"Check": "Physical robot", "Status": "Simulation only"},
        {"Check": "Obstacle events", "Status": str(st.session_state.obstacles_detected)},
        {"Check": "Successful reroutes", "Status": str(st.session_state.successful_reroutes)},
        {"Check": "Emergency stop", "Status": "ACTIVE" if st.session_state.emergency_stopped else "Ready"},
    ]
    st.dataframe(rows, hide_index=True, width="stretch")
    st.warning(
        "Arduino motion commands remain disabled. Sensor diagnostics report raw readings only "
        "and are unverified until the sensors are connected and tested. A software STOP is not "
        "a substitute for a physical motor-power emergency stop."
    )
    ports = list(list_ports.comports())
    labels = [port.device for port in ports]
    if not labels:
        st.info("No serial devices detected. Connect the controller by USB to inspect it.")
    connection = st.session_state.robot_connection
    connected = connection is not None and connection.is_open
    st.write(f"**Controller connection:** {'Connected' if connected else 'Disconnected'}")
    if connected:
        if st.button("Request controller status"):
            try:
                connection.send_command("STATUS")
                response = connection.read_message()
                st.session_state.last_robot_message = response or "No response before timeout."
            except (serial.SerialException, OSError, UnicodeError) as error:
                st.session_state.last_robot_message = f"Status request failed: {error}"
                st.error(st.session_state.last_robot_message)
        if st.button("Read raw sensor values"):
            try:
                connection.send_command("SENSORS")
                response = connection.read_message()
                st.session_state.last_robot_message = response or "No sensor response before timeout."
            except (serial.SerialException, OSError, UnicodeError) as error:
                st.session_state.last_robot_message = f"Sensor read failed: {error}"
                st.error(st.session_state.last_robot_message)
        if st.button("Send software STOP"):
            try:
                connection.send_command("STOP")
                response = connection.read_message()
                st.session_state.last_robot_message = response or "STOP sent; no acknowledgement received."
            except (serial.SerialException, OSError, UnicodeError) as error:
                st.session_state.last_robot_message = f"STOP command failed: {error}"
                st.error(st.session_state.last_robot_message)
        if st.button("Disconnect controller"):
            connection.close()
            st.session_state.robot_connection = None
            st.session_state.last_robot_message = "Disconnected"
            st.rerun()
    elif labels:
        selected_port = st.selectbox("USB serial port", labels)
        if st.button("Connect for diagnostics only"):
            try:
                st.session_state.robot_connection = RobotConnection(selected_port)
                st.session_state.last_robot_message = "Serial port opened; motion remains disabled."
                st.rerun()
            except (serial.SerialException, OSError, ValueError) as error:
                st.session_state.robot_connection_error = str(error)
        if st.session_state.robot_connection_error:
            st.error(f"Could not connect: {st.session_state.robot_connection_error}")
    st.write(f"**Latest controller message:** {st.session_state.last_robot_message}")


def main() -> None:
    st.set_page_config(
        page_title="OPTIROUTE | Warehouse Control Center",
        page_icon="🤖",
        layout="wide",
    )
    _initialize_state()
    st.markdown(
        """
        <style>
        .block-container {padding-top: 1.4rem; max-width: 1500px;}
        [data-testid="stMetric"] {
            background: #101b2a; border: 1px solid #26384d;
            border-radius: 12px; padding: 14px 18px;
        }
        </style>
        """,
        unsafe_allow_html=True,
    )
    st.title("OPTIROUTE")
    st.caption("Intelligent Warehouse Management & Autonomous Navigation")
    with st.sidebar:
        st.markdown("### Control Center")
        page = st.radio("Workspace", PAGES, key="workspace")
        st.divider()
        st.caption("Route objective weights")
        distance_weight = st.slider(
            "Distance",
            0.0,
            1.0,
            0.5,
            0.05,
            key="weight_distance",
        )
        time_weight = st.slider(
            "Travel time",
            0.0,
            1.0,
            0.3,
            0.05,
            key="weight_time",
        )
        congestion_weight = st.slider(
            "Congestion",
            0.0,
            1.0,
            0.2,
            0.05,
            key="weight_congestion",
        )
        st.caption("Simulation-first · 5 × 5 warehouse")
    graph = _make_graph()
    if distance_weight + time_weight + congestion_weight == 0:
        st.error("At least one route weight must be greater than zero.")
        st.stop()
    weights = CostWeights(distance_weight, time_weight, congestion_weight)
    if page == "Overview":
        _overview(graph, weights)
    elif page == "Warehouse Map":
        _warehouse_map(graph)
    elif page == "Route Optimisation":
        _route_optimisation(graph, weights)
    elif page == "Warehouse Simulation":
        _warehouse_simulation(graph, weights)
    elif page == "Robot Control":
        _robot_control(graph, weights)
    elif page == "Delivery Management":
        _delivery_management(graph, weights)
    elif page == "Analytics":
        _analytics(graph)
    else:
        _diagnostics()


if __name__ == "__main__":
    main()
