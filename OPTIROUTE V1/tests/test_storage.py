import tempfile
import unittest
from pathlib import Path

from optiroute.storage import (
    load_orders,
    load_routes,
    load_traversals,
    save_order,
    save_route,
    save_traversal,
)


class StorageTests(unittest.TestCase):
    def test_orders_routes_and_traversals_persist(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "optiroute.db"
            order = {"id": "ORD-001", "status": "Pending", "destinations": [24]}
            route = {
                "time": "2026-11-03T12:00:00+05:30",
                "algorithm": "A*",
                "purpose": "Single delivery",
                "start": 0,
                "destination": 24,
                "nodes": [0, 1, 6, 11, 16, 21, 22, 23, 24],
                "distance_m": 1.6,
                "distance_efficiency": 100.0,
                "travel_time_s": 16.0,
                "turns": 2,
                "cost": 8.0,
                "expanded_nodes": 9,
                "elapsed_ms": 0.1,
                "success": True,
            }

            save_order(order, path)
            order["status"] = "Completed"
            save_order(order, path)
            save_route(route, path)
            save_traversal((0, 1), path)

            self.assertEqual(load_orders(path), [order])
            self.assertEqual(load_routes(path)[0]["nodes"], route["nodes"])
            self.assertEqual(load_routes(path)[0]["start"], 0)
            self.assertEqual(load_traversals(path), [(0, 1)])


if __name__ == "__main__":
    unittest.main()
