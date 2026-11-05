"""The map data must match the design document before any logic can trust it."""
import pytest
from fleet_core.site_map import SiteMapError, load_site_map


@pytest.fixture(scope="module")
def site():
    return load_site_map()


def test_map_matches_the_design_document(site):
    assert (site.cols, site.rows) == (12, 7)
    assert len(site.zones) == 16
    assert len(set(site.zone_of)) == 30
    work, outbound = site.stations["WORK"], site.stations["OUTBOUND"]
    assert (work.dock, work.queue, work.server, work.arms, work.action) == (
        (5, 3), ((4, 3), (3, 3)), "workcell", ("arm1", "arm2"), "fill")
    assert (outbound.dock, outbound.queue, outbound.server, outbound.arms, outbound.action) == (
        (9, 3), ((9, 4), (9, 5)), "arm3", ("arm3",), "swap")


def test_every_route_cell_belongs_to_exactly_one_zone(site):
    route_cells = {c for r in site.routes.values() for c in r.cells}
    assert route_cells == set(site.zone_of)
    assert sum(len(cells) for cells in site.zones.values()) == len(site.zone_of)


def test_round_trip_lengths_match_the_design_document(site):
    def cells(name):
        return len(site.routes[name].cells)

    assert cells("to_outbound") == 9
    assert cells("to_work") == 17
    assert cells("waiting_loop") == 6


def test_plan_follows_the_one_way_lanes(site):
    path = site.plan((9, 3), "", (2, 0))
    assert path[:3] == [(9, 3), (9, 2), (9, 1)]
    assert path[-3:] == [(4, 0), (3, 0), (2, 0)]
    assert (9, 4) not in path, "must not reverse into the outbound queue"


def test_plan_from_outbound_dock_to_work_passes_outside_the_waiting_cells(site):
    path = site.plan((9, 3), "", (5, 3))
    assert len(path) == 17
    assert not set(path) & set(site.parking)


def test_plan_into_the_waiting_cells_uses_the_link(site):
    path = site.plan((9, 3), "", (3, 4))
    assert path[-7:] == [(2, 3), (2, 4), (2, 5), (2, 6), (3, 6), (3, 5), (3, 4)]


def test_plan_from_a_waiting_cell_rejoins_before_the_work_queue(site):
    assert site.plan((3, 5), "wait", (5, 3)) == [(3, 5), (3, 4), (3, 3), (4, 3), (5, 3)]


def test_plan_from_past_the_commit_zone_to_waiting_laps_through_both_docks(site):
    path = site.plan((3, 3), "ret", (2, 4))
    assert path[:3] == [(3, 3), (4, 3), (5, 3)]
    assert (9, 3) in path
    assert path[-2:] == [(2, 3), (2, 4)]


def test_plan_to_unreachable_cell_raises(site):
    with pytest.raises(SiteMapError):
        site.plan((9, 3), "", (5, 1))


def test_cell_pose_is_the_centre_of_the_cell(site):
    x, y = site.cell_pose((2, 0))
    assert x == pytest.approx(2.5 * site.cell_size)
    assert y == pytest.approx(0.5 * site.cell_size)


def test_distance_to_zone_is_zero_inside_and_grows_outside(site):
    x, y = site.cell_pose((9, 4))
    assert site.distance_to_zone(x, y, "Q-OUT-1") == 0.0
    # Centre of (9,3) is half a cell beyond the edge of (9,4).
    x, y = site.cell_pose((9, 3))
    assert site.distance_to_zone(x, y, "Q-OUT-1") == pytest.approx(site.cell_size / 2)


def test_cell_override_changes_pose_only_for_that_cell(tmp_path):
    base = load_site_map()
    text = base.source_text.replace("cell_overrides: {}", 'cell_overrides: {"8,2": [1.0, 2.0]}')
    path = tmp_path / "site.yaml"
    path.write_text(text, encoding="utf-8")
    site = load_site_map(path)
    assert site.cell_pose((8, 2)) == (1.0, 2.0)
    assert site.cell_pose((8, 3)) == base.cell_pose((8, 3))


def test_nearest_cell_finds_the_cell_under_a_pose(site):
    x, y = site.cell_pose((3, 5))
    assert site.nearest_cell(x + 0.02, y - 0.03) == (3, 5)


@pytest.mark.parametrize(
    "old, new, message",
    [
        ("P-6: [[3,4]]", "P-6: [[3,4],[3,5]]", "more than one zone"),
        ("P-6: [[3,4]]", "P-6: []", "no zone"),
        ("waiting_loop: {flow: wait, cells: [[2,4],[2,5]",
         "waiting_loop: {flow: wait, cells: [[2,4],[4,5]", "adjacent"),
        ("obstacles: [[3,1]", "obstacles: [[8,5],[3,1]", "obstacle"),
    ],
)
def test_broken_map_is_rejected_when_loaded(tmp_path, old, new, message):
    text = load_site_map().source_text
    assert old in text
    path = tmp_path / "site.yaml"
    path.write_text(text.replace(old, new), encoding="utf-8")
    with pytest.raises(SiteMapError, match=message):
        load_site_map(path)
