"""자유바닥 지도: 빈 바닥을 이미지(또는 사각형들)에서 읽고 바닥 전체를 cell_m 칸으로 나눈다.

칸 (i, j)는 열 i, 행 j다. 칸 원점은 지도 원점(이미지 왼쪽 아래)이고 칸 이름은 "c<i>_<j>"다.
  - free: 칸 전체가 빈 바닥이다.
  - clear: 로봇 중심이 서도 되는 칸. 칸 사각형 어디서든 막힌 곳까지 반지름 + wall_margin_m(없으면 cell_margin_m) 이상.
    계획기는 점이 어느 칸에 드는지만 보므로 칸 중심이 아니라 칸 전체가 안전해야 한다.
고정 선(fixed_lines)은 좁은 곳·도킹 접근·출고 앞에서 따라야 하는 선과 방향이다. 점 순서가 방향이다.
고정 선은 통째로 잠근다(양방향 통로에서 마주 오지 않게). rolling_lines에 적은 선은 한 방향만 다니는
선이라 바닥처럼 잠금 앞 거리만큼만 굴려 잠그고, 접근선 한 대 규칙도 쓰지 않는다(뒤 로봇이 선 위에 줄 선다).
narrow_lines에 적은 고정 선은 clear 칸이 없는 좁은 틈(두 팔 사이 도크)을 지난다. 로봇이 선 위로만
달리므로 칸이 아니라 선의 모든 점이 막힌 곳까지 반지름 + wall_margin_m 이상이면 된다(픽셀 1/4 간격으로
재고, 점마다 그 간격의 반만큼 더 요구한다). 그 선이 지나는 칸 중 clear가 아닌 칸이 line_floor다(틈 안).
자유 경로는 틈에 들어가지 않는다: 틈에는 그 선으로만 들어가고, 틈 안의 로봇은 선을 따라 앞으로만 나온다.
좁은 선은 rolling_lines에도 있어야 한다(통째 잠금과 접근선 한 대 규칙은 틈에서 따져 보지 않았다).
shortcuts(있으면) {지름길: 선}: 같은 도크로 가는 선 대신 탈 수 있는 짧은 선. 시작과 끝이
그 선과 같고 사이에서 갈라졌다 다시 합친다. 둘 다 rolling_lines다. 로봇은 그 선으로 출발해
갈림목 앞에서 고른다(관제).
robot_half_width_m(있으면): 차동구동 로봇은 옆으로 가지 않으니 곧게 달리는 동안은 반폭 + cell_margin_m만
잠그고, 제자리에서 돌 수 있는 곳(출발점, 끝점, 크게 꺾이는 점 앞)만 반지름 + cell_margin_m 원으로 잠근다.
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


def _off_line(points, p) -> float:
    """Distance from p to the polyline points."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:], strict=False):
        dx, dy = bx - ax, by - ay
        u = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy or 1.0)
        u = min(1.0, max(0.0, u))
        best = min(best, math.dist(p, (ax + u * dx, ay + u * dy)))
    return best


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
        # wall clearance of clear cells; robot-to-robot spacing keeps using cell_margin_m
        self.wall_margin_m = self._read_wall_margin(data.get("wall_margin_m", self.cell_margin_m))
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
        self.rolling_lines = self._read_rolling(data.get("rolling_lines") or [])
        self.narrow_lines = self._read_narrow(data.get("narrow_lines") or [])
        # shortcut -> the line into the same dock it may be taken instead of, and where along the
        # shortcut it leaves that line and joins it again (m)
        self.shortcuts = self._read_shortcuts(data.get("shortcuts") or {})
        self.shortcut_parts = {name: self._shortcut_parts(name, base)
                               for name, base in self.shortcuts.items()}
        self.robot_half_width_m = self._read_half_width(data.get("robot_half_width_m"))
        # beside a straight run only: robot_half_width_m + side_margin_m (default cell_margin_m)
        self.side_margin_m = self._read_side_margin(data.get("side_margin_m", self.cell_margin_m))
        self._ids = {}
        self._cells = {}
        for i in range(self.cols):
            for j in range(self.rows):
                self._ids[(i, j)] = f"c{i}_{j}"
                self._cells[f"c{i}_{j}"] = (i, j)
        self.free = frozenset(c for c in self._ids if self._cell_free(c))
        wall = self.robot_radius_m + self.wall_margin_m
        self.clear = frozenset(c for c in self.free if self._box_gap_to_wall(c) >= wall)
        # cells under each narrow line that are not clear (the gap): driven along that line only
        self.line_floor_of = {name: frozenset(
            self.cell_of(x, y) for x, y in self._samples(self.fixed_lines[name], self.px_m / 4)
        ) - self.clear for name in sorted(self.narrow_lines)}
        self.line_floor = frozenset().union(*self.line_floor_of.values())
        self._check()

    # ---- reading ------------------------------------------------------------------------

    def _read_narrow(self, names) -> frozenset:
        if not isinstance(names, list) or any(not isinstance(n, str) or n not in self.fixed_lines
                                              for n in names):
            raise FloorMapError(f"narrow_lines {names!r} must list fixed line names")
        if not set(names) <= self.rolling_lines:
            raise FloorMapError(f"narrow_lines {names!r} must be rolling lines too")
        return frozenset(names)

    def _read_shortcuts(self, value) -> dict:
        lines = self.fixed_lines
        if (not isinstance(value, dict)
                or any(not isinstance(k, str) or not isinstance(v, str) or k not in lines
                       or v not in lines or k == v or v in value for k, v in value.items())
                or len(set(value.values())) < len(value)):
            raise FloorMapError(f"shortcuts {value!r} must map a fixed line to another fixed "
                                f"line that is no shortcut itself, one shortcut a line")
        if not set(value) | set(value.values()) <= self.rolling_lines:
            # taken on the way without stopping: a line locked whole has a turn to wait for
            raise FloorMapError(f"shortcuts {value!r} must be rolling lines, as their lines")
        return dict(value)

    def _shortcut_parts(self, name: str, base: str) -> tuple[float, float]:
        """Along shortcut name: where it leaves line base (the end of the part it runs on base
        from its start) and where it joins base again (the start of the part it runs on base to
        its end). It starts and ends where base does and leaves it between."""
        short, line = self.fixed_lines[name], self.fixed_lines[base]
        if math.dist(short.start, line.start) > 1e-9 or math.dist(short.end, line.end) > 1e-9:
            raise FloorMapError(f"shortcut {name} must start and end where {base} does")
        pts = short.points

        def on_base(k):  # part k of the shortcut runs on base
            a, b = pts[k], pts[k + 1]
            steps = max(1, math.ceil(math.dist(a, b) / 0.005))
            return all(_off_line(line.points, (a[0] + (b[0] - a[0]) * i / steps,
                                               a[1] + (b[1] - a[1]) * i / steps)) <= 1e-6
                       for i in range(steps + 1))

        cum = [0.0]
        for a, b in zip(pts, pts[1:], strict=False):
            cum.append(cum[-1] + math.dist(a, b))
        fork = 0
        while fork < len(pts) - 1 and on_base(fork):
            fork += 1
        merge = len(pts) - 1
        while merge > 0 and on_base(merge - 1):
            merge -= 1
        if cum[fork] >= cum[merge] - 1e-9:
            raise FloorMapError(f"shortcut {name} never leaves {base}")
        return cum[fork], cum[merge]

    def _samples(self, line, step: float) -> list:
        """Points along a fixed line, at most step apart, the ends of every part included."""
        found = []
        for a, b in zip(line.points, line.points[1:], strict=False):
            steps = max(1, math.ceil(math.dist(a, b) / step - 1e-9))
            found += [(a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                      for k in range(steps + 1)]
        return found

    def _on_narrow_line(self, pose) -> bool:
        """pose is a point of a narrow line (a dock at its end or along it)."""
        for name in self.narrow_lines:
            points = self.fixed_lines[name].points
            for (ax, ay), (bx, by) in zip(points, points[1:], strict=False):
                dx, dy = bx - ax, by - ay
                u = ((pose[0] - ax) * dx + (pose[1] - ay) * dy) / (dx * dx + dy * dy or 1.0)
                u = min(1.0, max(0.0, u))
                if math.dist(pose, (ax + u * dx, ay + u * dy)) <= 1e-6:
                    return True
        return False

    def _read_rolling(self, names) -> frozenset:
        if not isinstance(names, list) or any(n not in self.fixed_lines for n in names
                                              if isinstance(n, str)) \
                or not all(isinstance(n, str) for n in names):
            raise FloorMapError(f"rolling_lines {names!r} must list fixed line names")
        return frozenset(names)

    def _read_half_width(self, value) -> float | None:
        """차동구동 로봇의 반폭. 없으면 None: 늘 반지름 원으로 잠근다."""
        if value is None:
            return None
        if (isinstance(value, bool) or not isinstance(value, int | float)
                or not math.isfinite(value) or not 0 < value <= self.robot_radius_m):
            raise FloorMapError(f"robot_half_width_m {value!r} must be a number in "
                                f"(0, robot_radius_m {self.robot_radius_m}]")
        return float(value)

    def _read_side_margin(self, value) -> float:
        """곧게 달리는 동안 반폭 바깥으로 더 잠그는 여유. 0 이상, cell_margin_m 이하."""
        if (isinstance(value, bool) or not isinstance(value, int | float)
                or not math.isfinite(value) or not 0 <= value <= self.cell_margin_m):
            raise FloorMapError(f"side_margin_m {value!r} must be a number in "
                                f"[0, cell_margin_m {self.cell_margin_m}]")
        return float(value)

    @staticmethod
    def _read_wall_margin(value) -> float:
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise FloorMapError(f"wall_margin_m {value!r} is not a number")
        try:
            margin = float(value)
        except OverflowError:
            raise FloorMapError(f"wall_margin_m {value!r} is too large") from None
        if not math.isfinite(margin) or margin < 0:
            raise FloorMapError(f"wall_margin_m {value!r} must be a finite number >= 0")
        return margin

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
        wall = self.robot_radius_m + self.wall_margin_m
        for name, pose in self.docks.items():
            if self.cell_of(*pose) not in self.clear and not (
                    self._on_narrow_line(pose) and self._gap_to_wall(*pose, wall) >= wall - 1e-9):
                raise FloorMapError(f"{name} at {pose} is not on clear floor")
        for name, pose in self.parking:
            if self.cell_of(*pose) not in self.clear:
                raise FloorMapError(f"{name} at {pose} is not on clear floor")
        for line in self.fixed_lines.values():
            if len(line.points) < 2:
                raise FloorMapError(f"fixed line {line.name} needs two points")
            if line.name in self.narrow_lines:
                # the distance to a wall changes by at most the way between two points, so
                # points step apart that keep wall + step / 2 keep the whole line at wall
                step = self.px_m / 4
                need = wall + step / 2
                for x, y in self._samples(line, step):
                    if (self.cell_of(x, y) not in self.free
                            or self._gap_to_wall(x, y, need) < need - 1e-9):
                        raise FloorMapError(f"narrow line {line.name} is nearer than "
                                            f"{wall:.2f} m to a wall at ({x:.2f}, {y:.2f})")
                continue
            for x, y in self._samples(line, self.cell_m / 2):
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

    def _gap_to_wall(self, x: float, y: float, limit: float = math.inf) -> float:
        """(x, y)에서 가장 가까운 막힌 픽셀이나 지도 끝까지의 거리. limit보다 먼 것은 찾지 않고
        limit을 돌려준다(가까운지만 볼 때)."""
        x0, y0 = self.origin
        width, height = self.cols * self.cell_m, self.rows * self.cell_m
        best = min(x - x0, y - y0, x0 + width - x, y0 + height - y, limit)
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
    # a link may point elsewhere only where floor_map.yaml itself does (an install of links)
    home = DEFAULT_FLOOR.resolve().parent
    if Path(name).name != name or not path.is_file() or path.resolve().parent != home:
        raise FloorMapError(f"floor_map {name!r} is not a file in {DEFAULT_FLOOR.parent}")
    floor = load_floor_map(path)
    extra = sorted(set(floor.docks) - set(site.stations))
    if extra:
        raise FloorMapError(f"{path.name} does not fit the lane map: docks {extra} are no station"
                            " of the lane map (set floor_map in the lane map)")
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
