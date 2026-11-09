"""자유바닥 지도: 빈 바닥을 이미지(또는 사각형들)에서 읽고 바닥 전체를 cell_m 칸으로 나눈다.

칸 (i, j)는 열 i, 행 j다. 칸 원점은 지도 원점(이미지 왼쪽 아래)이고 칸 이름은 "c<i>_<j>"다.
  - free: 칸 전체가 빈 바닥이다.
  - clear: 로봇 중심이 서도 되는 칸. 칸 사각형 어디서든 가장 가까운 막힌 곳까지 반지름 + 여유 이상.
    계획기는 점이 어느 칸에 드는지만 보므로 칸 중심이 아니라 칸 전체가 안전해야 한다.
고정 선(fixed_lines)은 좁은 곳·도킹 접근·출고 앞에서 따라야 하는 선과 방향이다. 점 순서가 방향이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_FLOOR = Path(__file__).parents[2] / "data" / "floor_map.yaml"


class FloorMapError(ValueError):
    pass


@dataclass(frozen=True)
class FixedLine:
    name: str
    points: tuple  # ((x, y), ...) in driving order

    @property
    def start(self):
        return self.points[0]

    @property
    def end(self):
        return self.points[-1]

    def length(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:], strict=False))


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    """8비트 binary PGM(P5, maxval 255)만 읽는다."""
    data = path.read_bytes()
    fields, at = [], 0
    while len(fields) < 4:  # magic, width, height, maxval; '#' comments allowed
        while at < len(data) and data[at:at + 1].isspace():
            at += 1
        if at >= len(data):
            raise FloorMapError(f"{path.name}: the header ends early")
        if data[at:at + 1] == b"#":
            at = data.find(b"\n", at)
            if at < 0:
                raise FloorMapError(f"{path.name}: the header ends early")
            continue
        end = at
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        fields.append(data[at:end])
        at = end
    if fields[0] != b"P5":
        raise FloorMapError(f"{path.name}: only binary PGM (P5) is read")
    if fields[3] != b"255":
        raise FloorMapError(f"{path.name}: only maxval 255 is read, not {fields[3].decode()}")
    width, height = int(fields[1]), int(fields[2])
    pixels = data[at + 1:at + 1 + width * height]  # one whitespace byte ends the header
    if len(pixels) != width * height:
        raise FloorMapError(f"{path.name}: {len(pixels)} pixels, expected {width * height}")
    return width, height, pixels


class FloorMap:
    def __init__(self, data: dict, base: Path | None = None, source_text: str = ""):
        self.source_text = source_text
        self.cell_m = float(data["cell_m"])
        self.robot_radius_m = float(data["robot_radius_m"])
        self.cell_margin_m = float(data["cell_margin_m"])
        self._occupied_px, self.px_m, self.origin, size = self._read_occupancy(data, base)
        per = self.cell_m / self.px_m
        if abs(per - round(per)) > 1e-9 or round(per) < 1:
            raise FloorMapError(f"cell_m {self.cell_m} must be a whole number of "
                                f"{self.px_m} m pixels")
        self.cols = int(round(size[0] / self.cell_m, 9))
        self.rows = int(round(size[1] / self.cell_m, 9))
        self.docks = {name: tuple(p) for name, p in (data.get("docks") or {}).items()}
        self.parking = [(spot["id"], (float(spot["x"]), float(spot["y"])))
                        for spot in data.get("parking") or []]
        self.fixed_lines = {name: FixedLine(name, tuple(tuple(map(float, p)) for p in points))
                            for name, points in (data.get("fixed_lines") or {}).items()}
        self._ids = {}
        self._cells = {}
        for i in range(self.cols):
            for j in range(self.rows):
                self._ids[(i, j)] = f"c{i}_{j}"
                self._cells[f"c{i}_{j}"] = (i, j)
        self.free = frozenset(c for c in self._ids if self._cell_free(c))
        body = self.robot_radius_m + self.cell_margin_m
        self.clear = frozenset(c for c in self.free if self._box_gap_to_wall(c) >= body)
        self._check()

    # ---- reading ------------------------------------------------------------------------

    @staticmethod
    def _read_occupancy(data, base):
        """막힌 픽셀 집합 (px, py)(아래에서 위로), 픽셀 크기, 원점, 지도 크기."""
        if "image" in data:
            path = Path(data["image"])
            if not path.is_absolute():
                path = (base or Path.cwd()) / path
            width, height, pixels = _read_pgm(path)
            res = float(data["resolution"])
            occupied_thresh = float(data.get("occupied_thresh", 0.65))
            free_thresh = float(data.get("free_thresh", 0.25))
            negate = bool(data.get("negate", 0))
            occupied = set()
            for row in range(height):
                for col in range(width):
                    value = pixels[row * width + col]
                    p = value / 255.0 if negate else (255 - value) / 255.0
                    if p > free_thresh or p >= occupied_thresh:  # unknown counts as occupied
                        occupied.add((col, height - 1 - row))
            origin = tuple(map(float, data["origin"][:2]))
            return frozenset(occupied), res, origin, (width * res, height * res)
        # 사각형 바닥: bounds [x0, y0, x1, y1] 안이 비고 boxes가 막혔다
        res = float(data.get("resolution", 0.05))
        x0, y0, x1, y1 = map(float, data["bounds"])
        boxes = [tuple(map(float, b)) for b in data.get("boxes") or []]
        width, height = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
        occupied = set()
        for col in range(width):
            for row in range(height):
                px0, py0 = x0 + col * res, y0 + row * res
                # a pixel the box overlaps at all is occupied, so thin boxes are not lost
                if any(b[0] < px0 + res and px0 < b[2] and b[1] < py0 + res and py0 < b[3]
                       for b in boxes):
                    occupied.add((col, row))
        return frozenset(occupied), res, (x0, y0), (width * res, height * res)

    def _check(self) -> None:
        for name, pose in [*self.docks.items(), *self.parking]:
            if self.cell_of(*pose) not in self.clear:
                raise FloorMapError(f"{name} at {pose} is not on clear floor")
        for line in self.fixed_lines.values():
            if len(line.points) < 2:
                raise FloorMapError(f"fixed line {line.name} needs two points")
            for a, b in zip(line.points, line.points[1:], strict=False):
                steps = max(1, math.ceil(math.dist(a, b) / (self.cell_m / 2)))
                for k in range(steps + 1):
                    x, y = a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps
                    if self.cell_of(x, y) not in self.clear:
                        raise FloorMapError(f"fixed line {line.name} leaves clear floor at "
                                            f"({x:.2f}, {y:.2f})")

    def _cell_free(self, cell) -> bool:
        i, j = cell
        per = int(round(self.cell_m / self.px_m))
        return not any((i * per + a, j * per + b) in self._occupied_px
                       for a in range(per) for b in range(per))

    def _box_gap_to_wall(self, cell) -> float:
        """칸 사각형에서 가장 가까운 막힌 픽셀이나 지도 끝까지의 거리."""
        x0, y0 = self.origin
        cx0, cy0 = x0 + cell[0] * self.cell_m, y0 + cell[1] * self.cell_m
        cx1, cy1 = cx0 + self.cell_m, cy0 + self.cell_m
        best = min(cx0 - x0, cy0 - y0, x0 + self.cols * self.cell_m - cx1,
                   y0 + self.rows * self.cell_m - cy1)
        reach = int(math.ceil(best / self.px_m)) + 1
        per = int(round(self.cell_m / self.px_m))
        pc, pr = cell[0] * per, cell[1] * per
        for col in range(pc - reach, pc + per + reach):
            for row in range(pr - reach, pr + per + reach):
                if (col, row) in self._occupied_px:
                    bx, by = x0 + col * self.px_m, y0 + row * self.px_m
                    dx = max(bx - cx1, 0.0, cx0 - (bx + self.px_m))
                    dy = max(by - cy1, 0.0, cy0 - (by + self.px_m))
                    best = min(best, math.hypot(dx, dy))
        return best

    def _gap_to_wall(self, x: float, y: float) -> float:
        """(x, y)에서 가장 가까운 막힌 픽셀이나 지도 끝까지의 거리."""
        x0, y0 = self.origin
        width, height = self.cols * self.cell_m, self.rows * self.cell_m
        best = min(x - x0, y - y0, x0 + width - x, y0 + height - y)
        reach = int(math.ceil(best / self.px_m)) + 1
        pc, pr = int((x - x0) / self.px_m), int((y - y0) / self.px_m)
        for col in range(pc - reach, pc + reach + 1):
            for row in range(pr - reach, pr + reach + 1):
                if (col, row) in self._occupied_px:
                    bx, by = x0 + col * self.px_m, y0 + row * self.px_m
                    dx = max(bx - x, 0.0, x - (bx + self.px_m))
                    dy = max(by - y, 0.0, y - (by + self.px_m))
                    best = min(best, math.hypot(dx, dy))
        return best

    # ---- cells --------------------------------------------------------------------------

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor((x - self.origin[0]) / self.cell_m + 1e-9),
                math.floor((y - self.origin[1]) / self.cell_m + 1e-9))

    def centre(self, cell) -> tuple[float, float]:
        i, j = cell
        return (self.origin[0] + (i + 0.5) * self.cell_m, self.origin[1] + (j + 0.5) * self.cell_m)

    def cell_id(self, cell) -> str:
        return self._ids[cell]

    def cell_at(self, cell_id: str):
        """칸 이름의 (i, j), 모르는 이름이면 None."""
        return self._cells.get(cell_id)

    def inside(self, cell) -> bool:
        return cell in self._ids

    def box_distance(self, cell, x: float, y: float) -> float:
        """(x, y)에서 칸 사각형까지의 거리. 안이면 0."""
        x0 = self.origin[0] + cell[0] * self.cell_m
        y0 = self.origin[1] + cell[1] * self.cell_m
        dx = max(x0 - x, 0.0, x - (x0 + self.cell_m))
        dy = max(y0 - y, 0.0, y - (y0 + self.cell_m))
        return math.hypot(dx, dy)

    def bounds(self) -> tuple[float, float, float, float]:
        return (self.origin[0], self.origin[1], self.origin[0] + self.cols * self.cell_m,
                self.origin[1] + self.rows * self.cell_m)

    def occupied_pixels(self):
        return self._occupied_px


def load_floor_map(path=None) -> FloorMap:
    source = Path(path) if path else DEFAULT_FLOOR
    text = source.read_text(encoding="utf-8")
    return FloorMap(yaml.safe_load(text), source.parent, text)


def floor_map_for(site) -> FloorMap:
    """The floor map of a lane map: the file its floor_map key names in fleet_core/data, or the
    default floor_map.yaml when it names none.

    Every node reads the same lane map (map_file / lane_map), so every node gets the same floor.
    A floor whose docks or parking are not those of the lane map is refused: a lane map must not
    run on the floor of another arena.
    """
    name = getattr(site, "floor_map", "") or DEFAULT_FLOOR.name
    path = DEFAULT_FLOOR.parent / name
    if Path(name).name != name or not path.is_file():
        raise FloorMapError(f"floor_map {name!r} is not a file in {DEFAULT_FLOOR.parent}")
    floor = load_floor_map(path)
    for station_id, station in site.stations.items():
        dock = floor.docks.get(station_id)
        if dock is None or math.dist(dock, site.waypoint_pose(station.dock)) > 1e-9:
            raise FloorMapError(f"{path.name} does not fit the lane map: dock {station_id} is "
                                f"{dock}, the lane map has {site.waypoint_pose(station.dock)}"
                                " (set floor_map in the lane map)")
    spots = [(spot, site.waypoint_pose(spot)) for spot in site.parking]
    if [spot for spot, _ in floor.parking] != [spot for spot, _ in spots] or any(
            math.dist(a, b) > 1e-9 for (_, a), (_, b) in zip(floor.parking, spots, strict=True)):
        raise FloorMapError(f"{path.name} does not fit the lane map: parking {floor.parking}, "
                            f"the lane map has {spots} (set floor_map in the lane map)")
    return floor
