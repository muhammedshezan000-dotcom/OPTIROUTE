import unittest
from xml.etree import ElementTree

from optiroute.visualization import render_warehouse_svg
from optiroute.warehouse import WarehouseGraph


class WarehouseVisualizationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.graph = WarehouseGraph()

    def test_animation_moves_robot_through_each_tick_position(self) -> None:
        svg = render_warehouse_svg(
            self.graph,
            route=(0, 1, 2),
            start=0,
            destination=2,
            current=2,
            animation_path=(0, 1, 2),
        )

        ElementTree.fromstring(svg)
        self.assertIn('aria-label="Animated simulated robot"', svg)
        self.assertIn('path="M 52 52 L 164 52 L 276 52"', svg)
        self.assertIn('dur="0.90s"', svg)

    def test_animation_is_not_rendered_without_a_movement(self) -> None:
        svg = render_warehouse_svg(
            self.graph,
            route=(),
            start=0,
            destination=24,
            current=0,
        )

        self.assertNotIn("<animateMotion", svg)


if __name__ == "__main__":
    unittest.main()
