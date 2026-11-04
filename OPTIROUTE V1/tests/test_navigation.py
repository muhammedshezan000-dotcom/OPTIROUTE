import unittest

from optiroute.warehouse import WarehouseGraph
from robot.navigation import Heading, route_to_commands


class NavigationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.graph = WarehouseGraph()

    def test_route_commands_follow_heading_changes(self) -> None:
        commands = route_to_commands(self.graph, (0, 1, 6, 11))

        self.assertEqual(commands, ("RIGHT", "RIGHT", "FORWARD"))

    def test_route_commands_respect_initial_heading(self) -> None:
        commands = route_to_commands(
            self.graph, (0, 5), initial_heading=Heading.SOUTH
        )

        self.assertEqual(commands, ("FORWARD",))

    def test_non_adjacent_route_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            route_to_commands(self.graph, (0, 6))


if __name__ == "__main__":
    unittest.main()
