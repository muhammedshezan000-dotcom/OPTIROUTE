import unittest

from optiroute.delivery import optimize_delivery_order
from optiroute.warehouse import WarehouseGraph


class DeliveryPlanningTests(unittest.TestCase):
    def test_delivery_route_visits_each_stop_and_returns_best_plan(self) -> None:
        graph = WarehouseGraph()

        plan = optimize_delivery_order(graph, 0, (4, 20, 24))

        self.assertIsNotNone(plan)
        self.assertEqual(set(plan.stops), {4, 20, 24})
        self.assertEqual(plan.route[0], 0)
        self.assertEqual(plan.route[-1], plan.stops[-1])

    def test_delivery_planner_rejects_more_than_eight_stops(self) -> None:
        graph = WarehouseGraph()

        with self.assertRaises(ValueError):
            optimize_delivery_order(graph, 0, tuple(range(1, 10)))

    def test_delivery_planner_returns_none_for_unreachable_stop(self) -> None:
        graph = WarehouseGraph()
        barrier = tuple((row * 5 + 1, row * 5 + 2) for row in range(5))
        graph.set_blocked_edges(barrier)

        self.assertIsNone(optimize_delivery_order(graph, 0, (4,)))


if __name__ == "__main__":
    unittest.main()
