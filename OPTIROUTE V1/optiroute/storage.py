from __future__ import annotations

import json
import sqlite3
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator


DATABASE_PATH = Path(__file__).resolve().parent.parent / "data" / "optiroute.db"


def _connect(path: Path = DATABASE_PATH) -> sqlite3.Connection:
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    connection.row_factory = sqlite3.Row
    connection.execute(
        """
        CREATE TABLE IF NOT EXISTS routes (
            id INTEGER PRIMARY KEY,
            created_at TEXT NOT NULL,
            algorithm TEXT NOT NULL,
            purpose TEXT NOT NULL,
            start_node INTEGER NOT NULL,
            destination INTEGER NOT NULL,
            nodes_json TEXT NOT NULL,
            distance_m REAL NOT NULL,
            distance_efficiency REAL NOT NULL,
            travel_time_s REAL NOT NULL,
            turns INTEGER NOT NULL,
            cost REAL NOT NULL,
            expanded_nodes INTEGER NOT NULL,
            elapsed_ms REAL NOT NULL,
            success INTEGER NOT NULL
        )
        """
    )
    connection.execute(
        """
        CREATE TABLE IF NOT EXISTS orders (
            order_id TEXT PRIMARY KEY,
            payload_json TEXT NOT NULL
        )
        """
    )
    connection.execute(
        """
        CREATE TABLE IF NOT EXISTS traversals (
            id INTEGER PRIMARY KEY,
            nodes_json TEXT NOT NULL
        )
        """
    )
    return connection


@contextmanager
def _database(path: Path) -> Iterator[sqlite3.Connection]:
    connection = _connect(path)
    try:
        with connection:
            yield connection
    finally:
        connection.close()


def load_routes(path: Path = DATABASE_PATH) -> list[dict[str, Any]]:
    with _database(path) as connection:
        rows = connection.execute(
            "SELECT * FROM routes ORDER BY id"
        ).fetchall()
    routes = []
    for row in rows:
        route = dict(row)
        route["nodes"] = json.loads(route.pop("nodes_json"))
        route["start"] = route.pop("start_node")
        route["success"] = bool(route["success"])
        routes.append(route)
    return routes


def save_route(record: dict[str, Any], path: Path = DATABASE_PATH) -> None:
    with _database(path) as connection:
        connection.execute(
            """
            INSERT INTO routes (
                created_at, algorithm, purpose, start_node, destination, nodes_json,
                distance_m, distance_efficiency, travel_time_s, turns, cost,
                expanded_nodes, elapsed_ms, success
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                record["time"],
                record["algorithm"],
                record["purpose"],
                record["start"],
                record["destination"],
                json.dumps(record["nodes"]),
                record["distance_m"],
                record["distance_efficiency"],
                record["travel_time_s"],
                record["turns"],
                record["cost"],
                record["expanded_nodes"],
                record["elapsed_ms"],
                int(record["success"]),
            ),
        )


def load_orders(path: Path = DATABASE_PATH) -> list[dict[str, Any]]:
    with _database(path) as connection:
        rows = connection.execute(
            "SELECT payload_json FROM orders ORDER BY rowid"
        ).fetchall()
    return [json.loads(row["payload_json"]) for row in rows]


def save_order(order: dict[str, Any], path: Path = DATABASE_PATH) -> None:
    with _database(path) as connection:
        connection.execute(
            """
            INSERT INTO orders (order_id, payload_json) VALUES (?, ?)
            ON CONFLICT(order_id) DO UPDATE SET payload_json = excluded.payload_json
            """,
            (order["id"], json.dumps(order)),
        )


def load_traversals(path: Path = DATABASE_PATH) -> list[tuple[int, ...]]:
    with _database(path) as connection:
        rows = connection.execute(
            "SELECT nodes_json FROM traversals ORDER BY id"
        ).fetchall()
    return [tuple(json.loads(row["nodes_json"])) for row in rows]


def save_traversal(nodes: tuple[int, ...], path: Path = DATABASE_PATH) -> None:
    with _database(path) as connection:
        connection.execute(
            "INSERT INTO traversals (nodes_json) VALUES (?)",
            (json.dumps(nodes),),
        )
