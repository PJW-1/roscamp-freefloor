"""좁은 통로와 두 줄 통로(floor_keep_right). 지도 모양에서만 구한다.

통로 칸: 한 축(x 또는 y) 방향으로 뻗은 빈 칸(clear) 중, 그 축에 수직으로 이어진 빈 칸이
max_cells개 이하인 칸. 맞닿은 통로 칸끼리(같은 축) 한 통로다.
두 줄 열: 통로를 가로지르는 한 줄(열)에서 양 끝 빈 칸을 중심으로 반지름 radius 원이 닿는 칸이
서로 겹치지 않는 열(옆 열의 반대쪽 끝과도). 거기서는 두 로봇이 양 끝 줄로 나란히 지난다.
줄: 두 줄 열마다 낮은 쪽(lo, 수직 좌표가 작은 쪽) 끝 칸과 높은 쪽(hi) 끝 칸. 옆 열과 끝이 한 칸
어긋나면 바깥쪽 열에 안쪽 칸을 더해 줄이 4방향으로 이어지게 한다(대각선 이동은 모서리를 못 자른다).
"""
from __future__ import annotations

from dataclasses import dataclass

from fleet_core.traffic.floor.cells import disk_cells


@dataclass(frozen=True)
class Passage:
    axis: int  # 0: x 방향으로 뻗음(열은 i), 1: y 방향(열은 j)
    cells: frozenset  # 통로 칸 (i, j)
    pair: frozenset  # 두 줄 열의 칸
    lo: frozenset  # 낮은 쪽 줄 칸
    hi: frozenset  # 높은 쪽 줄 칸


def _cell(axis: int, k: int, j: int) -> tuple[int, int]:
    """축 axis의 열 k, 수직 칸 j."""
    return (k, j) if axis == 0 else (j, k)


def _runs(floor, axis: int) -> dict:
    """열 k -> [(lo, hi)]: 축에 수직으로 이어진 빈 칸 토막들."""
    cols: dict[int, list[int]] = {}
    for c in floor.clear:
        cols.setdefault(c[axis], []).append(c[1 - axis])
    out = {}
    for k, js in cols.items():
        js.sort()
        runs, start, prev = [], js[0], js[0]
        for j in js[1:]:
            if j != prev + 1:
                runs.append((start, prev))
                start = j
            prev = j
        runs.append((start, prev))
        out[k] = runs
    return out


def run_widths(floor) -> tuple[dict, dict]:
    """축마다 칸 -> 그 축에 수직으로 이어진 빈 칸 수."""
    out = []
    for axis in (0, 1):
        width = {}
        for k, runs in _runs(floor, axis).items():
            for lo, hi in runs:
                for j in range(lo, hi + 1):
                    width[_cell(axis, k, j)] = hi - lo + 1
        out.append(width)
    return out[0], out[1]


def passages(floor, radius: float, max_cells: int) -> list[Passage]:
    found = []
    for axis in (0, 1):
        runs = {k: [r for r in rs if r[1] - r[0] + 1 <= max_cells]
                for k, rs in _runs(floor, axis).items()}
        # 맞닿은 토막끼리 묶는다(옆 열과 수직 칸이 겹치면 맞닿음)
        seen, parts = set(), []
        for k in sorted(runs):
            for run in runs[k]:
                if (k, run) in seen:
                    continue
                part, todo = [], [(k, run)]
                seen.add((k, run))
                while todo:
                    kk, (lo, hi) = todo.pop()
                    part.append((kk, (lo, hi)))
                    for nk in (kk - 1, kk + 1):
                        for other in runs.get(nk, ()):
                            if other[0] <= hi and other[1] >= lo and (nk, other) not in seen:
                                seen.add((nk, other))
                                todo.append((nk, other))
                parts.append(part)
        for part in parts:
            found.append(_passage(floor, axis, dict(part), radius))
    return found


def _passage(floor, axis: int, cols: dict, radius: float) -> Passage:
    cells = frozenset(_cell(axis, k, j) for k, (lo, hi) in cols.items()
                      for j in range(lo, hi + 1))
    good = set(cols)
    while True:  # 두 줄이 겹치는 열을 빼고 줄을 다시 짓는다
        lo_lane, hi_lane = _lanes(axis, {k: cols[k] for k in good})
        bad = set()
        for k in good:
            near_lo = {c for c in lo_lane if abs(c[axis] - k) <= 1}
            near_hi = {c for c in hi_lane if abs(c[axis] - k) <= 1}
            disk_lo = set().union(*(disk_cells(floor, *floor.centre(c), radius) for c in near_lo))
            if any(disk_lo & disk_cells(floor, *floor.centre(c), radius) for c in near_hi):
                bad.add(k)
        if not bad:
            break
        good -= bad
    pair = frozenset(_cell(axis, k, j) for k in good for j in range(cols[k][0], cols[k][1] + 1))
    return Passage(axis, cells, pair, frozenset(lo_lane), frozenset(hi_lane))


def _lanes(axis: int, cols: dict) -> tuple[set, set]:
    lo_lane, hi_lane = set(), set()
    for k, (lo, hi) in cols.items():
        lo_lane.add(_cell(axis, k, lo))
        hi_lane.add(_cell(axis, k, hi))
        nxt = cols.get(k + 1)
        if nxt is None:
            continue
        # 낮은 쪽: 끝이 더 낮은(바깥) 열에 안쪽 칸을 더한다. 높은 쪽도 같다
        a, b = lo, nxt[0]
        if a < b:
            lo_lane.update(_cell(axis, k, j) for j in range(a + 1, b + 1))
        elif b < a:
            lo_lane.update(_cell(axis, k + 1, j) for j in range(b + 1, a + 1))
        a, b = hi, nxt[1]
        if a > b:
            hi_lane.update(_cell(axis, k, j) for j in range(b, a))
        elif b > a:
            hi_lane.update(_cell(axis, k + 1, j) for j in range(a, b))
    return lo_lane, hi_lane
