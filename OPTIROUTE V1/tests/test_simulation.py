import unittest

from optiroute.simulation import advance_simulation
from optiroute.warehouse import CostWeights, WarehouseGraph


class SimulationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.graph = WarehouseGraph()
        self.weights = CostWeights()

    def test_tick_advances_one_corridor_and_updates_heading(self) -> None:
        result = advance_simulation(self.graph, 0, 24, self.weights)

        self.assertEqual(result.position, 1)
        self.assertEqual(result.heading, "EAST")
        self.assertAlmostEqual(result.moved_distance_m, 0.2)
        self.assertFalse(result.arrived)

    def test_obstacle_tick_replans_around_blocked_corridor(self) -> None:
        result = advance_simulation(
            self.graph,
            0,
            24,
            self.weights,
            obstacle_probability=1.0,
        )

        self.assertEqual(result.obstacle, (0, 1))
        self.assertTrue(result.rerouted)
        self.assertNotEqual(result.position, 1)
        self.assertTrue(self.graph.edge(0, 1).blocked)

    def test_existing_full_barrier_stops_simulation(self) -> None:
        barrier = tuple((row * 5 + 1, row * 5 + 2) for row in range(5))

        result = advance_simulation(
            self.graph,
            0,
            4,
            self.weights,
            blocked_edges=barrier,
        )

        self.assertTrue(result.blocked)
        self.assertEqual(result.position, 0)

    def test_robot_already_at_destination_is_arrived(self) -> None:
        result = advance_simulation(self.graph, 12, 12, self.weights)

        self.assertTrue(result.arrived)
        self.assertEqual(result.position, 12)

    def test_obstacle_probability_is_validated(self) -> None:
        with self.assertRaises(ValueError):
            advance_simulation(
                self.graph,
                0,
                24,
                self.weights,
                obstacle_probability=1.1,
            )


if __name__ == "__main__":
    unittest.main()
