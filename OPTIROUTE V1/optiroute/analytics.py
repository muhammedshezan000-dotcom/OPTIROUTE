from __future__ import annotations

from collections import Counter


def estimate_congestion(
    traversed_routes: tuple[tuple[int, ...], ...],
) -> dict[tuple[int, int], float]:
    """Estimate relative congestion on corridors from observed route frequency."""
    counts: Counter[tuple[int, int]] = Counter()
    for route in traversed_routes:
        for left, right in zip(route, route[1:]):
            counts[(min(left, right), max(left, right))] += 1
    if not counts:
        return {}
    highest_count = max(counts.values())
    return {
        edge: min(10.0, count / highest_count * 10.0)
        for edge, count in counts.items()
    }
