import unittest

from optiroute.analytics import estimate_congestion


class CongestionAnalyticsTests(unittest.TestCase):
    def test_predictor_ranks_frequently_traversed_corridors(self) -> None:
        estimates = estimate_congestion(((0, 1, 2), (0, 1, 6), (0, 1, 2)))

        self.assertEqual(estimates[(0, 1)], 10.0)
        self.assertLess(estimates[(1, 2)], estimates[(0, 1)])

    def test_predictor_is_empty_without_observations(self) -> None:
        self.assertEqual(estimate_congestion(()), {})


if __name__ == "__main__":
    unittest.main()
