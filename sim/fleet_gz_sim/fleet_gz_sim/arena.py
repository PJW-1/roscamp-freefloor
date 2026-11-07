"""Gazebo 레인 경기장. 벽과 상자 하나의 기하에서 월드(SDF)와 지도(PGM, YAML)를 만든다.

월드 좌표 = 지도(map) 좌표 = lane_map.yaml 좌표다(원점 [0, 0]). 그래서 Gazebo의 실제
위치를 AMCL 위치, 관제 지점 좌표와 바로 비교할 수 있다. 레인은 바닥에 그림으로만 있다.

    python3 -m fleet_gz_sim.arena          # 패키지의 worlds/, maps/를 다시 만든다
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

CLEARANCE_M = 0.25  # 레인에서 벽·상자까지 최소 거리(안내: 레인 여유 0.25 m)
WALL_M = 0.05
WALL_HEIGHT_M = 0.30  # 라이다 높이(약 0.125 m)보다 높게
COARSE_M = 0.05  # costmap용 지도
FINE_M = 0.01  # AMCL용 지도(R15)
# 모든 좌표가 0.05의 배수라 두 해상도 모두 칸 경계가 벽 경계와 맞는다.
MAP_ORIGIN = (-0.15, 0.10)
MAP_SIZE = (4.00, 2.45)
INSIDE = (0.00, 0.25, 3.70, 2.40)  # 벽 안쪽 (x0, y0, x1, y1)
WORLD = "lane_arena"
OUTPUTS = ("worlds/lane_arena.sdf", "maps/lane_arena.yaml", "maps/lane_arena.pgm",
           "maps/lane_arena_fine.yaml", "maps/lane_arena_fine.pgm")


@dataclass(frozen=True)
class Box:
    name: str
    x0: float
    y0: float
    x1: float
    y1: float

    def contains(self, x: float, y: float) -> bool:
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1


def walls() -> list[Box]:
    x0, y0, x1, y1 = INSIDE
    t = WALL_M
    return [Box("wall_south", x0 - t, y0 - t, x1 + t, y0),
            Box("wall_north", x0 - t, y1, x1 + t, y1 + t),
            Box("wall_west", x0 - t, y0, x0, y1),
            Box("wall_east", x1, y0, x1 + t, y1)]


# 방이 대칭이면 AMCL이 헷갈리므로 벽에 상자를 붙인다. 모두 레인에서 CLEARANCE_M 넘게 떨어져 있다.
FEATURES = (Box("block_sw", 0.00, 0.25, 0.50, 0.50),
            Box("block_w", 0.00, 0.75, 0.15, 1.00),
            Box("block_se", 3.30, 0.25, 3.70, 0.55),
            Box("block_n", 2.60, 2.10, 3.20, 2.40))


def obstacles() -> list[Box]:
    return [*walls(), *FEATURES]


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


def min_clearance(lane_map) -> tuple[float, str, str]:
    """(가장 가까운 거리, 레인, 상자)."""
    return min((clearance(a, b, box), name, box.name)
               for a, b, name in lane_segments(lane_map) for box in obstacles())


def occupancy(resolution: float) -> list[list[bool]]:
    """칸 중심이 벽·상자 안이면 점유. 줄은 아래(y 작은 쪽)부터."""
    nx, ny = round(MAP_SIZE[0] / resolution), round(MAP_SIZE[1] / resolution)
    ox, oy = MAP_ORIGIN
    boxes = obstacles()
    return [[any(b.contains(ox + (i + 0.5) * resolution, oy + (j + 0.5) * resolution)
                 for b in boxes) for i in range(nx)] for j in range(ny)]


def pgm(resolution: float) -> bytes:
    grid = occupancy(resolution)
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    # PGM 첫 줄은 지도 위쪽(y 큰 쪽). 점유 0(검정), 빈칸 254
    return header + b"".join(bytes(0 if c else 254 for c in row) for row in reversed(grid))


def map_yaml(name: str, resolution: float) -> str:
    ox, oy = MAP_ORIGIN
    return (f"# 생성 파일: python3 -m fleet_gz_sim.arena\n"
            f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def _num(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") if v else "0"


def _box_xml(box: Box) -> str:
    sx, sy = box.x1 - box.x0, box.y1 - box.y0
    cx, cy = (box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2
    pose = f"{_num(cx)} {_num(cy)} {_num(WALL_HEIGHT_M / 2)} 0 0 0"
    size = f"{_num(sx)} {_num(sy)} {_num(WALL_HEIGHT_M)}"
    return (f'      <collision name="{box.name}"><pose>{pose}</pose>'
            f"<geometry><box><size>{size}</size></box></geometry></collision>\n"
            f'      <visual name="{box.name}_visual"><pose>{pose}</pose>'
            f"<geometry><box><size>{size}</size></box></geometry>"
            "<material><ambient>0.55 0.45 0.35 1</ambient><diffuse>0.55 0.45 0.35 1</diffuse>"
            "</material></visual>\n")


def _lane_xml(a, b, name: str) -> str:
    """바닥에 그린 레인. 충돌 없음, 높이 1 mm라 라이다(약 0.125 m)에 보이지 않는다."""
    (ax, ay), (bx, by) = a, b
    length, yaw = math.hypot(bx - ax, by - ay), math.atan2(by - ay, bx - ax)
    pose = f"{_num((ax + bx) / 2)} {_num((ay + by) / 2)} 0.0005 0 0 {_num(yaw)}"
    return (f'      <visual name="lane_{name}"><pose>{pose}</pose>'
            f"<geometry><box><size>{_num(length)} 0.02 0.001</size></box></geometry>"
            "<material><ambient>0.2 0.3 0.8 1</ambient><diffuse>0.2 0.3 0.8 1</diffuse>"
            "</material></visual>\n")


def world_sdf(lane_map, render_engine: str = "ogre2") -> str:
    body = "".join(_box_xml(b) for b in obstacles())
    body += "".join(_lane_xml(a, b, name) for a, b, name in lane_segments(lane_map))
    return f"""<?xml version="1.0"?>
<!-- 생성 파일: python3 -m fleet_gz_sim.arena 로 다시 만든다. 손으로 고치지 않는다.
     월드 좌표 = 지도 좌표 = lane_map.yaml 좌표. 레인은 바닥 그림이고 벽·상자는 레인에서
     {CLEARANCE_M} m 넘게 떨어져 있다. -->
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
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


def generate(out: Path, lane_map, render_engine: str = "ogre2") -> None:
    value, lane, box = min_clearance(lane_map)
    if value < CLEARANCE_M:
        raise ValueError(f"{lane} is {value:.3f} m from {box} (needs {CLEARANCE_M} m)")
    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "maps").mkdir(parents=True, exist_ok=True)
    (out / "worlds" / f"{WORLD}.sdf").write_text(world_sdf(lane_map, render_engine),
                                                 encoding="utf-8")
    for name, resolution in ((WORLD, COARSE_M), (f"{WORLD}_fine", FINE_M)):
        (out / "maps" / f"{name}.yaml").write_text(map_yaml(name, resolution), encoding="utf-8")
        (out / "maps" / f"{name}.pgm").write_bytes(pgm(resolution))


def main(argv=None) -> None:
    from fleet_core.traffic.lane_map import load_lane_map
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1],
                        help="worlds/, maps/를 쓸 폴더(기본: 이 패키지 소스)")
    parser.add_argument("--lane-map", type=Path, default=None)
    args = parser.parse_args(argv)
    lane_map = load_lane_map(args.lane_map)
    generate(args.out, lane_map)
    value, lane, box = min_clearance(lane_map)
    print(f"wrote {', '.join(OUTPUTS)} under {args.out}; "
          f"closest lane-obstacle gap {value:.3f} m ({lane}, {box})")


if __name__ == "__main__":
    main()
