"""Gazebo 레인 경기장. 벽과 상자 하나의 기하에서 월드(SDF)와 지도(PGM, YAML)를 만든다.

월드 좌표 = 지도(map) 좌표 = lane_map.yaml 좌표다(원점 [0, 0]). 그래서 Gazebo의 실제
위치를 AMCL 위치, 관제 지점 좌표와 바로 비교할 수 있다. 레인은 바닥에 그림으로만 있다.
경기장(site)은 기본 lane_arena와 sites/<이름>.yaml 파일들이다.

    python3 -m fleet_gz_sim.arena                    # 패키지의 worlds/, maps/를 다시 만든다
    python3 -m fleet_gz_sim.arena --site warehouse   # sites/warehouse.yaml에서 만든다
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

CLEARANCE_M = 0.25  # 레인에서 벽·상자까지 최소 거리(안내: 레인 여유 0.25 m)
WALL_M = 0.05
WALL_HEIGHT_M = 0.30  # 라이다 높이(약 0.125 m)보다 높게
LIDAR_Z_M = 0.125
LIDAR_MARGIN_M = 0.05  # 상자는 라이다 평면보다 이만큼 더 높아야 한다
COARSE_M = 0.05  # costmap용 지도
FINE_M = 0.01  # AMCL용 지도(R15)
MAP_PAD_M = 0.15  # 지도는 벽 안쪽에서 이만큼씩 넓다(벽 0.05 m가 들어간다)
WORLD = "lane_arena"  # 모든 경기장의 월드 이름(launch가 이 이름으로 부른다)
PKG = Path(__file__).resolve().parents[1]
LANE_MAPS = PKG.parent / "fake_robots" / "maps"
DEFAULT_HEIGHT_M = {"wall": WALL_HEIGHT_M, "block": WALL_HEIGHT_M, "rack": 0.45, "table": 0.30,
                    "pillar": 0.60, "arm": 0.20}
COLOURS = {"wall": "0.55 0.45 0.35", "block": "0.55 0.45 0.35", "rack": "0.27 0.42 0.62",
           "table": "0.72 0.53 0.32", "pillar": "0.60 0.60 0.60", "arm": "0.20 0.20 0.22"}
ARM_ORANGE = "0.95 0.50 0.10"


class SiteError(ValueError):
    pass


@dataclass(frozen=True)
class Box:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float
    kind: str = "block"
    height_m: float = WALL_HEIGHT_M

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


@dataclass(frozen=True)
class Site:
    name: str
    inside: tuple[float, float, float, float]  # 벽 안쪽 (x0, y0, x1, y1)
    features: tuple[Box, ...]
    lane_map: str = ""  # sim/fake_robots/maps/의 이름, ""는 기본 레인 지도

    @property
    def map_origin(self) -> tuple[float, float]:
        x0, y0, _, _ = self.inside
        return round(x0 - MAP_PAD_M, 4), round(y0 - MAP_PAD_M, 4)

    @property
    def map_size(self) -> tuple[float, float]:
        x0, y0, x1, y1 = self.inside
        return round(x1 - x0 + 2 * MAP_PAD_M, 4), round(y1 - y0 + 2 * MAP_PAD_M, 4)

    def outputs(self) -> tuple[str, ...]:
        n = self.name
        return (f"worlds/{n}.sdf", f"maps/{n}.yaml", f"maps/{n}.pgm",
                f"maps/{n}_fine.yaml", f"maps/{n}_fine.pgm")


INSIDE = (0.00, 0.25, 3.70, 2.40)
# 방이 대칭이면 AMCL이 헷갈리므로 벽에 상자를 붙인다. 모두 레인에서 CLEARANCE_M 넘게 떨어져 있다.
FEATURES = (Box("block_sw", 0.00, 0.25, 0.50, 0.50),
            Box("block_w", 0.00, 0.75, 0.15, 1.00),
            Box("block_se", 3.30, 0.25, 3.70, 0.55),
            Box("block_n", 2.60, 2.10, 3.20, 2.40))
LANE_ARENA = Site(WORLD, INSIDE, FEATURES)
# 모든 좌표가 0.05의 배수라 두 해상도 모두 칸 경계가 벽 경계와 맞는다.
MAP_ORIGIN = LANE_ARENA.map_origin
MAP_SIZE = LANE_ARENA.map_size
OUTPUTS = LANE_ARENA.outputs()


def walls(site: Site = LANE_ARENA) -> list[Box]:
    x0, y0, x1, y1 = site.inside
    t = WALL_M
    return [Box("wall_south", x0 - t, y0 - t, x1 + t, y0, "wall"),
            Box("wall_north", x0 - t, y1, x1 + t, y1 + t, "wall"),
            Box("wall_west", x0 - t, y0, x0, y1, "wall"),
            Box("wall_east", x1, y0, x1 + t, y1, "wall")]


def obstacles(site: Site = LANE_ARENA) -> list[Box]:
    return [*walls(site), *site.features]


class _UniqueKeyLoader(yaml.SafeLoader):
    """safe_load that refuses a key given twice in one mapping (safe_load keeps the last)."""


def _unique_mapping(loader, node, deep=False):
    keys = [loader.construct_object(k, deep=deep) for k, _ in node.value]
    for key in keys:
        if keys.count(key) > 1:
            raise yaml.constructor.ConstructorError(
                None, None, f"duplicate key {key!r}", node.start_mark)
    return loader.construct_mapping(node, deep)


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def load_site(path) -> Site:
    """sites/<이름>.yaml을 읽는다. 잘못된 파일은 SiteError."""
    path = Path(path)
    try:
        data = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader) or {}
    except yaml.constructor.ConstructorError as e:
        raise SiteError(f"{path}: {e.problem}") from e
    if not isinstance(data, dict):
        raise SiteError(f"{path}: expected a mapping")
    extra = set(data) - {"lane_map", "bounds_m", "boxes"}
    if extra:
        raise SiteError(f"{path}: unknown keys {sorted(extra)}")
    try:
        x0, y0, x1, y1 = (float(v) for v in data["bounds_m"])
    except (KeyError, TypeError, ValueError) as e:
        raise SiteError(f"{path}: bounds_m must be [x0, y0, x1, y1]") from e
    if not all(math.isfinite(v) for v in (x0, y0, x1, y1)):
        raise SiteError(f"{path}: bounds_m must be finite numbers")
    if x1 <= x0 or y1 <= y0:
        raise SiteError(f"{path}: bounds_m is empty")
    site = Site(path.stem, (x0, y0, x1, y1), (), str(data.get("lane_map") or ""))
    names = {b.name for b in walls(site)}
    features = []
    for raw in data.get("boxes") or []:
        box = _site_box(path, raw)
        if box.name in names:
            raise SiteError(f"{path}: box {box.name} named twice")
        if box.x0 < x0 or box.y0 < y0 or box.x1 > x1 or box.y1 > y1:
            raise SiteError(f"{path}: box {box.name} is outside bounds_m")
        names.add(box.name)
        features.append(box)
    return Site(site.name, site.inside, tuple(features), site.lane_map)


def _site_box(path: Path, raw) -> Box:
    if not isinstance(raw, dict):
        raise SiteError(f"{path}: box {raw!r} is not a mapping")
    extra = set(raw) - {"name", "kind", "x0", "y0", "x1", "y1", "height_m"}
    if extra:
        raise SiteError(f"{path}: box {raw.get('name')} has unknown keys {sorted(extra)}")
    kind = raw.get("kind")
    if kind not in DEFAULT_HEIGHT_M:
        raise SiteError(f"{path}: box {raw.get('name')} has kind {kind!r}, "
                        f"not one of {sorted(DEFAULT_HEIGHT_M)}")
    try:
        box = Box(str(raw["name"]), *(float(raw[k]) for k in ("x0", "y0", "x1", "y1")), kind,
                  float(raw.get("height_m", DEFAULT_HEIGHT_M[kind])))
    except (KeyError, TypeError, ValueError) as e:
        raise SiteError(f"{path}: box {raw.get('name')} needs name, x0, y0, x1, y1") from e
    if not all(math.isfinite(v) for v in (box.x0, box.y0, box.x1, box.y1, box.height_m)):
        raise SiteError(f"{path}: box {box.name} needs finite numbers")
    if box.x1 <= box.x0 or box.y1 <= box.y0:
        raise SiteError(f"{path}: box {box.name} is empty")
    if box.height_m < LIDAR_Z_M + LIDAR_MARGIN_M:
        raise SiteError(f"{path}: box {box.name} is {box.height_m} m high, lidar sees up to "
                        f"{LIDAR_Z_M} m (needs {LIDAR_Z_M + LIDAR_MARGIN_M} m)")
    return box


def lane_segments(lane_map) -> list[tuple[tuple[float, float], tuple[float, float], str]]:
    """방향을 뺀 레인 선분(양방향 쌍은 하나). (a, b, 이름)."""
    seen, out = set(), []
    for lane in lane_map.lanes.values():
        key = frozenset((lane.a, lane.b))
        if key not in seen:
            seen.add(key)
            out.append(((lane.ax, lane.ay), (lane.bx, lane.by), f"{lane.a}-{lane.b}"))
    return out


def _point_segment(p, a, b) -> float:
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _crosses(a, b, c, d) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def clearance(a, b, box: Box) -> float:
    """선분 ab와 상자 사이 최소 거리. 닿거나 겹치면 0."""
    corners = [(box.x0, box.y0), (box.x1, box.y0), (box.x1, box.y1), (box.x0, box.y1)]
    edges = list(zip(corners, corners[1:] + corners[:1], strict=True))
    if box.contains(*a) or box.contains(*b) or any(_crosses(a, b, c, d) for c, d in edges):
        return 0.0
    gaps = [_point_segment(c, a, b) for c in corners]
    gaps += [_point_segment(p, c, d) for p in (a, b) for c, d in edges]
    return min(gaps)


def min_clearance(lane_map, site: Site = LANE_ARENA) -> tuple[float, str, str]:
    """(가장 가까운 거리, 레인, 상자)."""
    return min((clearance(a, b, box), name, box.name)
               for a, b, name in lane_segments(lane_map) for box in obstacles(site))


def occupancy(resolution: float, site: Site = LANE_ARENA) -> list[list[bool]]:
    """벽·상자가 조금이라도(칸의 백만분의 1 넘게) 걸친 칸은 점유. 줄은 아래(y 작은 쪽)부터.
    격자에 맞춘 상자는 칸 중심으로 판정할 때와 같고, 얇거나 어긋난 상자도 지도에서
    사라지지 않는다."""
    nx, ny = round(site.map_size[0] / resolution), round(site.map_size[1] / resolution)
    ox, oy = site.map_origin
    grid = [[False] * nx for _ in range(ny)]
    hair = 1e-6  # 칸 단위

    def cells(lo: float, hi: float, origin: float, n: int) -> range:
        return range(max(0, math.floor((lo - origin) / resolution + hair)),
                     min(n, math.ceil((hi - origin) / resolution - hair)))

    for b in obstacles(site):
        for j in cells(b.y0, b.y1, oy, ny):
            for i in cells(b.x0, b.x1, ox, nx):
                grid[j][i] = True
    return grid


def pgm(resolution: float, site: Site = LANE_ARENA) -> bytes:
    grid = occupancy(resolution, site)
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    # PGM 첫 줄은 지도 위쪽(y 큰 쪽). 점유 0(검정), 빈칸 254
    return header + b"".join(bytes(0 if c else 254 for c in row) for row in reversed(grid))


def _command(site: Site, xml: bool = False) -> str:
    if site == LANE_ARENA:
        return "python3 -m fleet_gz_sim.arena"
    # XML 주석 안에는 "--"를 쓸 수 없다
    return f"python3 -m fleet_gz_sim.arena ({site.name}.yaml)" if xml else (
        f"python3 -m fleet_gz_sim.arena --site {site.name}")


def map_yaml(name: str, resolution: float, site: Site = LANE_ARENA) -> str:
    ox, oy = site.map_origin
    return (f"# 생성 파일: {_command(site)}\n"
            f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def _num(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") if v else "0"


def _material(colour: str) -> str:
    return f"<material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material>"


def _visual(name: str, pose: str, geometry: str, colour: str) -> str:
    return (f'      <visual name="{name}"><pose>{pose}</pose>'
            f"<geometry>{geometry}</geometry>{_material(colour)}</visual>\n")


def _box_xml(box: Box) -> str:
    sx, sy = box.x1 - box.x0, box.y1 - box.y0
    cx, cy = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2
    pose = f"{_num(cx)} {_num(cy)} {_num(box.height_m / 2)} 0 0 0"
    size = f"{_num(sx)} {_num(sy)} {_num(box.height_m)}"
    return (f'      <collision name="{box.name}"><pose>{pose}</pose>'
            f"<geometry><box><size>{size}</size></box></geometry></collision>\n"
            + _visual(f"{box.name}_visual", pose, f"<box><size>{size}</size></box>",
                      COLOURS[box.kind]))


def _arm_xml(box: Box, site: Site) -> str:
    """멈춰 있는 로봇 팔 모형. 상자는 받침(충돌 있음)이고 그 위 부품은 그림뿐이다.
    짧은 변 k에 비례: 회전판, 세운 위팔, 팔꿈치, 방 가운데로 30도 숙인 아래팔, 집게."""
    k = min(box.x1 - box.x0, box.y1 - box.y0)
    cx, cy, z = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2, box.height_m
    x0, y0, x1, y1 = site.inside
    yaw = math.atan2((y0 + y1) / 2 - cy, (x0 + x1) / 2 - cx)
    dark, cyl = COLOURS["arm"], "<cylinder><radius>{}</radius><length>{}</length></cylinder>"

    def at(x, y, zz, pitch=0.0, rot=0.0):
        return f"{_num(x)} {_num(y)} {_num(zz)} 0 {_num(pitch)} {_num(rot)}"

    turn_h, upper, fore, tilt = 0.15 * k, 1.2 * k, 1.0 * k, math.radians(30)
    elbow = z + turn_h + upper
    # 아래팔 축: 수평 cos(tilt), 아래로 sin(tilt)
    dx, dy, dz = math.cos(tilt) * math.cos(yaw), math.cos(tilt) * math.sin(yaw), -math.sin(tilt)
    out = _box_xml(box)
    out += _visual(f"{box.name}_turntable", at(cx, cy, z + turn_h / 2),
                   cyl.format(_num(0.4 * k), _num(turn_h)), ARM_ORANGE)
    out += _visual(f"{box.name}_upper", at(cx, cy, z + turn_h + upper / 2),
                   cyl.format(_num(0.12 * k), _num(upper)), ARM_ORANGE)
    out += _visual(f"{box.name}_elbow", at(cx, cy, elbow),
                   f"<sphere><radius>{_num(0.16 * k)}</radius></sphere>", dark)
    out += _visual(f"{box.name}_fore",
                   at(cx + dx * fore / 2, cy + dy * fore / 2, elbow + dz * fore / 2,
                      math.pi / 2 + tilt, yaw),
                   cyl.format(_num(0.1 * k), _num(fore)), ARM_ORANGE)
    size = f"{_num(0.2 * k)} {_num(0.25 * k)} {_num(0.2 * k)}"
    out += _visual(f"{box.name}_gripper",
                   at(cx + dx * fore, cy + dy * fore, elbow + dz * fore, 0.0, yaw),
                   f"<box><size>{size}</size></box>", dark)
    return out


def _lane_xml(a, b, name: str) -> str:
    """바닥에 그린 레인. 충돌 없음, 높이 1 mm라 라이다(약 0.125 m)에 보이지 않는다."""
    (ax, ay), (bx, by) = a, b
    length, yaw = math.hypot(bx - ax, by - ay), math.atan2(by - ay, bx - ax)
    pose = f"{_num((ax + bx) / 2)} {_num((ay + by) / 2)} 0.0005 0 0 {_num(yaw)}"
    return (f'      <visual name="lane_{name}"><pose>{pose}</pose>'
            f"<geometry><box><size>{_num(length)} 0.02 0.001</size></box></geometry>"
            "<material><ambient>0.2 0.3 0.8 1</ambient><diffuse>0.2 0.3 0.8 1</diffuse>"
            "</material></visual>\n")


# Gazebo는 물리 한 스텝마다 /clock을 내고, use_sim_time 노드는 그때마다 깬다. 1 ms(1 kHz)에서는
# 핑키 3대 실행의 Python 노드가 코어를 50~90 %씩 써서 실시간의 0.3~0.5로 돌았다(2026-11-08).
PHYSICS_STEP_S = 0.004


def world_sdf(lane_map, render_engine: str = "ogre2", site: Site = LANE_ARENA) -> str:
    body = "".join(_arm_xml(b, site) if b.kind == "arm" else _box_xml(b)
                   for b in obstacles(site))
    body += "".join(_lane_xml(a, b, name) for a, b, name in lane_segments(lane_map))
    return f"""<?xml version="1.0"?>
<!-- 생성 파일: {_command(site, xml=True)} 로 다시 만든다. 손으로 고치지 않는다.
     월드 좌표 = 지도 좌표 = lane_map.yaml 좌표. 레인은 바닥 그림이고 벽·상자는 레인에서
     {CLEARANCE_M} m 넘게 떨어져 있다. -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="4ms" type="ignored">
      <max_step_size>{PHYSICS_STEP_S}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="arena">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
  </world>
</sdf>
"""


def generate(out: Path, lane_map, render_engine: str = "ogre2", site: Site = LANE_ARENA) -> None:
    value, lane, box = min_clearance(lane_map, site)
    if value < CLEARANCE_M - 1e-9:  # 0.25 m exactly is allowed despite float rounding
        raise ValueError(f"{lane} is {value:.3f} m from {box} (needs {CLEARANCE_M} m)")
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "maps").mkdir(parents=True, exist_ok=True)
    (out / "worlds" / f"{site.name}.sdf").write_text(world_sdf(lane_map, render_engine, site),
                                                     encoding="utf-8")
    for name, resolution in ((site.name, COARSE_M), (f"{site.name}_fine", FINE_M)):
        (out / "maps" / f"{name}.yaml").write_text(map_yaml(name, resolution, site),
                                                   encoding="utf-8")
        (out / "maps" / f"{name}.pgm").write_bytes(pgm(resolution, site))


def main(argv=None) -> None:
    from fleet_core.traffic.lane_map import load_lane_map
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=PKG,
                        help="worlds/, maps/를 쓸 폴더(기본: 이 패키지 소스)")
    parser.add_argument("--site", default=None,
                        help="sites/<이름>.yaml(기본: 내장 lane_arena)")
    parser.add_argument("--lane-map", type=Path, default=None,
                        help="기본: 경기장 파일의 lane_map")
    args = parser.parse_args(argv)
    site = load_site(PKG / "sites" / f"{args.site}.yaml") if args.site else LANE_ARENA
    path = args.lane_map
    if path is None and site.lane_map:
        path = LANE_MAPS / f"{site.lane_map}.yaml"
    lane_map = load_lane_map(path)
    generate(args.out, lane_map, site=site)
    value, lane, box = min_clearance(lane_map, site)
    print(f"wrote {', '.join(site.outputs())} under {args.out}; "
          f"closest lane-obstacle gap {value:.3f} m ({lane}, {box})")


if __name__ == "__main__":
    main()
