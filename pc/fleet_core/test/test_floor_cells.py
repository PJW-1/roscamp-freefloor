"""몸이 쓰는 칸과 칸 잠금(P1). 칸 잠금은 구역 잠금(ZoneLocks)과 같은 읽기 API를 낸다."""
import pytest
from fleet_core.traffic.deadlock import find_cycle, wait_graph
from fleet_core.traffic.floor.cells import CellLocks, disk_cells, swept_cells
from fleet_core.traffic.floor.floor_map import FloorMap


@pytest.fixture
def floor():
    return FloorMap({"bounds": [0, 0, 2, 1], "cell_m": 0.1, "robot_radius_m": 0.08,
                     "cell_margin_m": 0.05})


def test_a_disk_takes_every_cell_it_touches(floor):
    cells = disk_cells(floor, 0.55, 0.55, 0.13)
    # centre cell plus its four neighbours (0.13 > 0.05) and the diagonals (0.05*sqrt2 < 0.13)
    assert floor.cell_of(0.55, 0.55) in cells
    assert len(cells) == 9
    for cell in cells:
        assert floor.box_distance(cell, 0.55, 0.55) <= 0.13
    for i in range(floor.cols):
        for j in range(floor.rows):
            if (i, j) not in cells:
                assert floor.box_distance((i, j), 0.55, 0.55) > 0.13


def test_a_disk_stops_at_the_map_edge(floor):
    cells = disk_cells(floor, 0.02, 0.02, 0.13)
    assert all(floor.inside(c) for c in cells)


def test_a_sweep_covers_the_disk_at_every_point_of_the_line(floor):
    line = [(0.3, 0.3), (1.2, 0.3), (1.5, 0.7)]
    cells = swept_cells(floor, line, 0.13)
    for k in range(101):
        s = k / 100
        if s < 0.5:
            x, y = 0.3 + 0.9 * s * 2, 0.3
        else:
            x, y = 1.2 + 0.3 * (s - 0.5) * 2, 0.3 + 0.4 * (s - 0.5) * 2
        assert disk_cells(floor, x, y, 0.13) <= cells


def test_a_one_point_sweep_is_the_disk(floor):
    assert swept_cells(floor, [(0.55, 0.55)], 0.13) == disk_cells(floor, 0.55, 0.55, 0.13)


# ---- locks ----------------------------------------------------------------------------


def locks():
    return CellLocks(lambda: 1.0)


def test_cells_are_taken_all_or_nothing():
    table = locks()
    assert table.acquire_all("a", {"c1_1", "c1_2"})
    assert not table.acquire_all("b", {"c3_3", "c1_2"})
    assert table.holder("c3_3") is None  # nothing of the failed extension was taken
    assert table.held_by("b") == []
    assert table.waiting_for("b") == "c1_2"
    assert table.blockers("b", "c1_2") == {"a"}


def test_a_robot_extends_over_its_own_cells():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    assert table.acquire_all("a", {"c1_1", "c1_2"})
    assert table.held_by("a") == ["c1_1", "c1_2"]
    assert table.waiting_for("a") is None


def test_release_and_wait_end_together():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    table.acquire_all("b", {"c1_1"})
    assert table.release("a", "c1_1")
    assert not table.release("a", "c1_1")  # not held any more
    assert table.acquire_all("b", {"c1_1"})
    assert table.waiting_for("b") is None


def test_the_snapshot_lists_only_held_or_waited_cells():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    table.acquire_all("b", {"c1_1", "c2_2"})
    snap = table.snapshot()
    assert set(snap) == {"c1_1"}
    assert snap["c1_1"] == {"holder": "a", "queue": ("b",), "acquired_at": 1.0}


def test_the_lane_wait_graph_finds_a_cell_cycle():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    table.acquire_all("b", {"c2_2"})
    table.acquire_all("a", {"c1_1", "c2_2"})
    table.acquire_all("b", {"c2_2", "c1_1"})
    graph = wait_graph(table)
    assert graph == {"a": {"b"}, "b": {"a"}}
    assert sorted(find_cycle(graph)) == ["a", "b"]


def test_cancelling_requests_clears_the_wait():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    table.acquire_all("b", {"c1_1"})
    table.cancel_all_requests("b")
    assert table.waiting_for("b") is None
    assert table.queue("c1_1") == ()


def test_force_acquire_takes_free_cells_only():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    assert not table.force_acquire("b", {"c1_1", "c1_2"})
    assert table.held_by("b") == []
    assert table.force_acquire("b", {"c1_2"})


def test_holders_around_names_other_holders():
    table = locks()
    table.acquire_all("a", {"c1_1"})
    assert table.holders_around("c1_1", "b") == {"c1_1": "a"}
    assert table.holders_around("c1_1", "a") == {}
    assert table.exists("c9_9") and not table.exists("zone")


def test_the_table_is_light_for_a_thousand_cells():
    table = locks()
    cells = {f"c{i}_{j}" for i in range(40) for j in range(25)}
    assert table.acquire_all("a", cells)
    assert len(table.held_by("a")) == 1000
    for cell in cells:
        table.release("a", cell)
    assert table.held_by("a") == [] and table.snapshot() == {}


# ---- Codex P1 review --------------------------------------------------------------------


def test_a_disk_touching_a_cell_edge_takes_that_cell(floor):
    # x - r lands exactly on the boundary x = 0.4 between cells 3 and 4
    cells = disk_cells(floor, 0.5, 0.55, 0.1)
    assert (3, 5) in cells


def test_a_generator_of_cells_is_taken_whole():
    table = locks()
    assert table.acquire_all("a", (c for c in ["c1_1", "c1_2"]))
    assert table.held_by("a") == ["c1_1", "c1_2"]
    assert table.force_acquire("b", (c for c in ["c2_1"]))
    assert table.held_by("b") == ["c2_1"]


def _sampled(floor, points, radius):
    """the first sweep: a disk of radius + step/2 at points a quarter cell apart"""
    from fleet_core.traffic.floor.cells import sample_line
    step = floor.cell_m / 4
    cells = set()
    for x, y in sample_line(points, step):
        cells |= disk_cells(floor, x, y, radius + step / 2)
    return cells


def test_the_sweep_is_exact_and_no_smaller_than_the_sampled_one(floor):
    """Each segment is swept in one pass (box to segment distance) instead of a disk every
    2.5 cm: the same cells or more, and none farther than the reach from the line."""
    import random

    from fleet_core.traffic.floor.cells import sample_line
    rng = random.Random(7)
    reach_extra = floor.cell_m / 8
    for _ in range(40):
        points = [(rng.uniform(0.3, 1.7), rng.uniform(0.3, 1.1)) for _ in range(rng.randint(2, 4))]
        got = swept_cells(floor, points, 0.13)
        assert _sampled(floor, points, 0.13) <= got
        fine = set()
        for x, y in sample_line(points, 0.002):
            fine |= disk_cells(floor, x, y, 0.13 + reach_extra + 0.002)
        assert got <= fine
