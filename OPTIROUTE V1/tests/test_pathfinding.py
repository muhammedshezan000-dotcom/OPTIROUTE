import unittest

from optiroute.pathfinding import astar, breadth_first, dijkstra
from optiroute.warehouse import CostWeights, WarehouseGraph


class PathfindingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.graph = WarehouseGraph()

    def test_open_grid_shortest_route_has_eight_steps(self) -> None:
        route = dijkstra(self.graph, 0, 24)

        self.assertIsNotNone(route)
        self.assertEqual(route.nodes[0], 0)
        self.assertEqual(route.nodes[-1], 24)
        self.assertEqual(len(route.nodes) - 1, 8)

    def test_astar_matches_dijkstra_cost(self) -> None:
        dijkstra_route = dijkstra(self.graph, 0, 24)
        astar_route = astar(self.graph, 0, 24)

        self.assertIsNotNone(dijkstra_route)
        self.assertIsNotNone(astar_route)
        self.assertAlmostEqual(dijkstra_route.total_cost, astar_route.total_cost)

    def test_breadth_first_finds_minimum_corridor_count(self) -> None:
        route = breadth_first(self.graph, 0, 24)

        self.assertIsNotNone(route)
        self.assertEqual(len(route.nodes) - 1, 8)

    def test_breadth_first_reports_weighted_cost_without_optimizing_it(self) -> None:
        weights = CostWeights(0.2, 0.1, 0.7)
        self.graph.set_congestion(0, 1, 10)
        self.graph.set_congestion(1, 2, 10)

        bfs_route = breadth_first(self.graph, 0, 2, weights)
        weighted_route = dijkstra(self.graph, 0, 2, weights)

        self.assertIsNotNone(bfs_route)
        self.assertIsNotNone(weighted_route)
        self.assertEqual(len(bfs_route.nodes) - 1, 2)
        self.assertGreater(bfs_route.total_cost, weighted_route.total_cost)

    def test_blocked_corridor_is_avoided(self) -> None:
        self.graph.set_blocked_edges(((0, 1),))
        route = dijkstra(self.graph, 0, 4)

        self.assertIsNotNone(route)
        self.assertEqual(route.nodes[1], 5)

    def test_full_barrier_reports_no_route(self) -> None:
        barrier = tuple((row * 5 + 1, row * 5 + 2) for row in range(5))
        self.graph.set_blocked_edges(barrier)

        self.assertIsNone(dijkstra(self.graph, 0, 4))

    def test_congestion_can_favor_a_longer_route(self) -> None:
        self.graph.set_congestion(0, 1, 10)
        self.graph.set_congestion(1, 2, 10)

        route = dijkstra(self.graph, 0, 2, CostWeights(0.2, 0.1, 0.7))

        self.assertIsNotNone(route)
        self.assertGreater(len(route.nodes) - 1, 2)

    def test_cost_weights_reject_invalid_values(self) -> None:
        with self.assertRaises(ValueError):
            CostWeights(0, 0, 0).normalized()
        with self.assertRaises(ValueError):
            CostWeights(-1, 1, 1).normalized()

    def test_non_adjacent_block_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.graph.set_blocked_edges(((0, 6),))

    def test_restricted_node_is_not_used_as_transit(self) -> None:
        self.graph.set_restricted_nodes((1,))
        route = dijkstra(self.graph, 0, 2)

        self.assertIsNotNone(route)
        self.assertNotIn(1, route.nodes)

    def test_corridor_speed_and_distance_are_configurable(self) -> None:
        self.graph.set_edge_properties(0, 1, distance_m=0.5, speed_limit_mps=0.25)

        self.assertAlmostEqual(self.graph.edge(0, 1).travel_time_s, 2.0)


if __name__ == "__main__":
    unittest.main()
