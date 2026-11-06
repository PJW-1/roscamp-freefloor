#!/usr/bin/env python3
"""실제 TrafficController를 레인 맵에서 호출 순서의 모든 경우로 전수 검사한다.

    python3 tools/model_check.py --map sim/fake_robots/maps/oneway_shortcut.yaml \
        --robots 3 --bundle off

상태는 로봇마다 임무 단계, 남은 경로, 진행 중인 goal, 위치와 구역 잠금, 열린 호출 수다.
주문은 상태에 없다. 매 단계에서 아래 중 하나가 일어나고, 이어서 코어가 tick을 한 번 돈다.

    open call       로봇을 지정하지 않은 dispatch(WORK), 로봇당 최대 하나
    cancel call     열린 호출에 대한 dispatch_cancel
    send X          dispatch(OUTBOUND, X)
    cancel X        X의 dispatch에 대한 dispatch_cancel
    release X       release(X)
    pass X          X가 레인을 따라 d를 지났다고 보고
    arrive X        X의 이동이 끝남
    docked X        X의 도킹이 끝남
    report X        X의 보고가 한 번 더 옴(--confirm이 1보다 클 때만)
    nothing         tick만

아무것도 바꾸지 못하는 호출(두 번째 dispatch, 풀 것이 없는 release)은 `nothing`과 같은 단계라 뺀다.
test_model_check_tool.py가 작은 맵에서 둘을 비교한다.

출력은 (states, deadlocks, starved)와 교착에 이르는 가장 짧은 경로 하나다.
  deadlock: 대기 순환이 있는 상태. 여기서는 시간이 멈춰 실제 코어의 후퇴가 오지 않으므로
    더 진행하지 않는다. 나중의 cancel이 풀 수도 있지만 그래도 센다.
  starved: (상태, 로봇) 쌍 중 로봇이 갈 곳이 남았는데 cancel이 아닌 어떤 사건 순서로도
    다시 움직일 수 없는 것. 교착의 모든 구성원이 포함된다. 0은 빠져나갈 길이 항상 있다는
    뜻이지 모든 일정이 그 길을 간다는 뜻이 아니다. 로봇을 계속 건너뛰는 사건 순서는
    세지 않으며, 그에 대한 방어는 zones.py의 FIFO다.

다루지 않는 것: 잠금 규칙 밖의 기하, 고장, 타이밍, 통신 두절. 위치는 "지점 위"
아니면 "레인에서 d를 막 지난 곳"뿐이다.

기본값은 세 번째가 아니라 첫 보고에서 구역을 푼다. 구역을 지났지만 아직 풀지 않은 로봇은
어떤 큐에도 없으므로 그 구역을 거치는 대기 순환에 들 수 없다. 이는 논증이며 기본 탐색이
증명하지는 않는다. --confirm 3은 그 보고들의 모든 순서도 탐색하지만 상태가 훨씬 많아진다.
둘 다 돌린 경우 작업 기록에 둘 다 있다.
`report`는 같은 위치를 반복한다. 구역 안으로 되돌아온 실제 보고는 횟수를 초기화해 해제를
늦출 뿐 탐색이 못 본 잠금 상태에 이르지 않는다. 이것도 논증이다.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing
import os
import sys
from array import array
from collections.abc import Iterable
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.common.params import load_params  # noqa: E402
from fleet_core.traffic.controller import TrafficController  # noqa: E402
from fleet_core.traffic.deadlock import find_cycle, wait_graph  # noqa: E402
from fleet_core.traffic.lane_map import load_lane_map  # noqa: E402
from fleet_core.traffic.mission import DOCK, MOVE, _Dispatch, _Goal  # noqa: E402

PAST = 0.01  # 로봇이 지나갈 때 d를 얼마나 넘어서 보고하는가
NAMED = 100  # 이름 지정 dispatch의 goal id는 여기서 시작해 열린 호출의 id와 겹치지 않는다
CHUNK = 500  # 프로세스 여럿이 일할 때 job 하나의 상태 수
ROBOT_FIELDS = 12  # 상태에서 로봇 하나를 나타내는 부분의 길이


class Model:
    """저장된 어떤 상태로도 맞출 수 있는 코어 하나. 한 단계에 객체 복사가 없다."""

    def __init__(self, site, robots: int, start=None, params=None, confirm: int = 1):
        self.confirm = confirm
        params = replace(params or load_params(), release_confirm_samples=confirm)
        self.site = site
        self.d = params.release_distance_m
        self.names = [f"r{i + 1}" for i in range(robots)]
        self.core = TrafficController(site, params, self.names, lambda: 0.0)
        _remember_plans(site)
        self._group = self._conflict_groups()
        spots = start or list(site.parking[-robots:])[::-1]
        self.pose = dict(zip(self.names, spots, strict=True))
        for name, node in self.pose.items():
            self.core.on_robot_state(name, *site.waypoint_pose(node), 0.0, "IDLE")
        refused = [m.name for m in self.core._missions.values() if not m.placed]
        if refused:
            raise ValueError(f"{refused} cannot start on {spots}: the lock is taken")
        self.core.on_run_mode("RUN")
        self._tick()
        self.initial = self.capture()
        self.event_list = self._all_events()
        self.event_code = {event: code for code, event in enumerate(self.event_list)}

    def _all_events(self) -> list[tuple]:
        found = [("nothing",), ("open call",), ("cancel call",)]
        for name in self.names:
            found += [(kind, name) for kind in ("send", "cancel", "release", "pass", "arrive",
                                                "docked", "report")]
        return found

    def _conflict_groups(self) -> dict:
        """요청 순서는 큐 안과 충돌 쌍 사이에서만 의미가 있다."""
        group = {zone: zone for zone in self.core.zones.snapshot()}
        for zone in group:
            for other in self.core.zones._conflicts[zone]:
                old, new = group[other], group[zone]
                for key, value in group.items():
                    if value == old:
                        group[key] = new
        return group

    # ---- state ----------------------------------------------------------------------------

    def capture(self) -> tuple:
        core = self.core
        robots = []
        for name in self.names:
            m = core._missions[name]
            first = self._first_needed(m)
            goal = None
            if m.goal:
                target = m.goal.target_idx - first if m.goal.kind == MOVE else 0
                goal = (m.goal.kind, target, m.goal.step)
            settle = None if m.settle_idx is None else m.settle_idx - first
            counted = ()
            if self.confirm > 1:
                counted = tuple(sorted((z, min(n, self.confirm)) for z, n in m.outside.items()))
            robots.append((m.phase, tuple(m.path[first:]), m.idx - first, m.dest, goal, settle,
                           m.called, m.dispatch.station_id if m.dispatch else None, m.hold,
                           m.await_pick, self.pose[name], counted))
        zones = core.zones
        holders = tuple(sorted((z, h) for z, h in zones._holder.items() if h))
        requests = sorted(zones._arrived.items(), key=lambda item: item[1])
        requests = sorted((self._group[z], rank, z, r)
                          for rank, ((z, r), _) in enumerate(requests))
        return (*robots, (holders, tuple((z, r) for _, _, z, r in requests)),
                len(core._open_calls))

    def _first_needed(self, m) -> int:
        """로봇 뒤쪽은 아직 잡고 있는 구역까지만 상태에 센다."""
        held = self.core.zones._held.get(m.name, ())
        for index in range(m.idx):
            if self.site.zone_of[m.path[index]] in held:
                return index
        return m.idx

    def restore(self, state: tuple) -> None:
        core = self.core
        for number, name in enumerate(self.names):
            m = core._missions[name]
            (m.phase, path, m.idx, m.dest, goal, m.settle_idx, m.called, station, m.hold,
             m.await_pick, self.pose[name], counted) = state[number]
            m.path = list(path)
            m.goal = _Goal(goal[0], 1, goal[1], goal[2]) if goal else None
            m.dispatch = _Dispatch(NAMED + number, station) if station else None
            m.x, m.y = self.xy(self.pose[name])
            m.outside = dict(counted)
            m.off_lane, m.off_lane_warned, m.wait_zone, m.attempts = 0, False, None, 0
        holders, queue = state[len(self.names)]
        zones = core.zones
        for zone in zones._holder:
            zones._holder[zone] = None
            zones._queue[zone] = []
        zones._held = {name: [] for name in self.names}
        for zone, holder in holders:
            zones._holder[zone] = holder
            zones._held[holder].append(zone)
        zones._arrived = {}
        for rank, (zone, robot) in enumerate(queue):
            zones._queue[zone].append(robot)
            zones._arrived[(zone, robot)] = rank
        zones._arrivals = len(queue)
        core._open_calls = list(range(state[-1]))
        core._deadlock_since, core._deadlock_acted, core._retreating = None, False, None
        core._out = []

    def xy(self, pose) -> tuple[float, float]:
        if isinstance(pose, tuple) and pose[0] == "lane":
            lane = self.site.lane(pose[1], pose[2])
            return lane.ax + lane.ux * (self.d + PAST), lane.ay + lane.uy * (self.d + PAST)
        return self.site.waypoint_pose(pose)

    def has_cycle(self) -> bool:
        return bool(find_cycle(wait_graph(self.core.zones)))

    # ---- events ---------------------------------------------------------------------------

    def _tick(self, reporter: str | None = None) -> None:
        """전원이 보고한다. 보고를 여러 번 세는 경우에는 움직인 로봇만 보고한다."""
        core = self.core
        reporters = self.names if self.confirm == 1 else [reporter] if reporter else []
        for name in reporters:
            m = core._missions[name]
            core.on_robot_state(name, m.x, m.y, 0.0, "IDLE")
        core.tick()

    def events(self, state: tuple) -> list[tuple]:
        """지금 일어날 수 있는 모든 사건. 상태를 먼저 복원해 두어야 한다."""
        core = self.core
        calls = state[-1]
        found = [("nothing",)]
        if calls < len(self.names):
            found.append(("open call",))
        if calls:
            found.append(("cancel call",))
        for name in self.names:
            m = core._missions[name]
            if m.dispatch:
                found.append(("cancel", name))
            else:
                found.append(("send", name))
            if m.hold:
                found.append(("release", name))
            if m.goal and m.goal.kind == MOVE:
                if m.idx < m.goal.target_idx and self.pose[name] != self._lane_pose(m):
                    found.append(("pass", name))
                found.append(("arrive", name))
            elif m.goal and m.goal.kind == DOCK:
                found.append(("docked", name))
            if self.confirm > 1:
                found.append(("report", name))
        return found

    @staticmethod
    def _lane_pose(m) -> tuple:
        return ("lane", m.path[m.idx], m.path[m.idx + 1])

    def apply(self, event: tuple) -> None:
        core = self.core
        kind = event[0]
        if kind == "open call":
            core.on_dispatch(len(core._open_calls), core._work)
        elif kind == "cancel call":
            core.on_dispatch_cancel(core._open_calls[-1])
        elif kind == "send":
            core.on_dispatch(NAMED + self.names.index(event[1]), core._outbound, event[1])
        elif kind == "cancel":
            core.on_dispatch_cancel(core._missions[event[1]].dispatch.goal_id)
        elif kind == "release":
            core.on_release(event[1])
        elif kind == "pass":
            m = core._missions[event[1]]
            self.pose[event[1]] = self._lane_pose(m)
            m.x, m.y = self.xy(self.pose[event[1]])
        elif kind == "arrive":
            m = core._missions[event[1]]
            self.pose[event[1]] = m.path[m.goal.target_idx]
            m.x, m.y = self.xy(self.pose[event[1]])
            core.on_move_result(event[1], m.goal.id, True, "")
        elif kind == "docked":
            core.on_dock_result(event[1], core._missions[event[1]].goal.id, True)
        self._tick(event[1] if kind in ("pass", "arrive", "report") else None)

    def expand(self, states: Iterable[tuple]) -> list:
        """상태마다 교착이면 None, 아니면 (사건 코드, 다음 상태) 쌍."""
        answers = []
        for state in states:
            self.restore(state)
            if self.has_cycle():
                answers.append(None)
                continue
            seen = set()
            successors = []
            for event in self.events(state):
                self.restore(state)
                self.apply(event)
                successor = self.capture()
                key = (successor, _is_cancel(event))
                if successor != state and key not in seen:
                    seen.add(key)
                    successors.append((self.event_code[event], successor))
            answers.append(successors)
        return answers


def _remember_plans(site) -> None:
    """경로는 맵에만 따르고 코어가 같은 것을 계속 요청하므로 기억해 둔다."""
    if "plan" in vars(site):
        return
    plans: dict = {}
    real_plan = site.plan

    def plan(a, b):
        found = plans.get((a, b))
        if found is None:
            found = plans[(a, b)] = real_plan(a, b)
        return found

    site.plan = plan


def _is_cancel(event: tuple) -> bool:
    return event[0] in ("cancel", "cancel call")


_worker_model: Model | None = None


def _start_worker(model_class, site, robots, start, params, confirm) -> None:
    global _worker_model
    _worker_model = model_class(site, robots, start, params, confirm)


def _work(states: list) -> list:
    return _worker_model.expand(states)


@dataclass
class Result:
    states: int
    deadlocks: int
    starved: int  # 탐색이 중간에 멈췄으면 -1
    example: list
    complete: bool
    local_states: int


class _Store:
    """알려진 모든 상태를 상태당 약 150바이트로 저장한다. 대부분의 부분이 자주 되풀이된다."""

    def __init__(self, robots: int):
        self.robots = robots
        self.part_ids: dict = {}
        self.parts: list = []
        self.columns = [array("I") for _ in range(robots + 1)]
        self.calls = array("B")
        self.index: dict[int, int] = {}

    def _part(self, value) -> int:
        found = self.part_ids.get(value)
        if found is None:
            found = self.part_ids[value] = len(self.parts)
            self.parts.append(value)
        return found

    def add(self, state: tuple) -> tuple[int, bool]:
        """상태 번호와 새 상태인지 여부를 돌려준다."""
        key = state[-1]
        ids = []
        for part in state[:-1]:
            part_id = self._part(part)
            ids.append(part_id)
            key = key << 32 | part_id
        found = self.index.get(key)
        if found is not None:
            return found, False
        number = len(self.index)
        self.index[key] = number
        for column, part_id in zip(self.columns, ids, strict=True):
            column.append(part_id)
        self.calls.append(state[-1])
        return number, True

    def state(self, number: int) -> tuple:
        parts = self.parts
        return (*(parts[column[number]] for column in self.columns), self.calls[number])

    def __len__(self) -> int:
        return len(self.index)


def check(site, robots: int = 3, start=None, params=None, max_states: int = 0, workers: int = 1,
          progress=None, model_class=Model, stop_at_first: bool = False,
          confirm: int = 1) -> Result:
    model = model_class(site, robots, start, params, confirm)
    store = _Store(robots)
    store.add(model.initial)
    parents = array("I", [0])
    parent_events = array("H", [0])
    edges_from, edges_to = array("I"), array("I")  # cancel 제외, starved 계산용
    dead: list[int] = []
    cancels = [_is_cancel(event) for event in model.event_list]
    frontier = [0]
    complete = True
    pool = None
    if workers > 1:
        pool = multiprocessing.get_context("fork").Pool(
            workers, _start_worker, (model_class, site, robots, start, params, confirm))
    try:
        while frontier:
            if max_states and len(store) >= max_states:
                complete = False
                break
            if stop_at_first and dead:
                complete = False
                break
            chunks = [frontier[i:i + CHUNK] for i in range(0, len(frontier), CHUNK)]
            jobs = ([store.state(n) for n in chunk] for chunk in chunks)
            answers = pool.imap(_work, jobs) if pool else map(model.expand, jobs)
            next_frontier = []
            for chunk, answer in zip(chunks, answers, strict=True):
                for node, successors in zip(chunk, answer, strict=True):
                    if successors is None:
                        dead.append(node)
                        continue
                    for code, successor in successors:
                        found, new = store.add(successor)
                        if new:
                            parents.append(node)
                            parent_events.append(code)
                            next_frontier.append(found)
                        if found != node and not cancels[code]:
                            edges_from.append(node)
                            edges_to.append(found)
            frontier = next_frontier
            if progress:
                progress(len(store), len(frontier), len(dead))
    finally:
        if pool:
            pool.terminate()
    starved = _count_starved(store, robots, edges_from, edges_to) if complete else -1
    example = _way_to(dead, parents, parent_events, model.event_list)
    robot_parts = sum(1 for part in store.parts if len(part) == ROBOT_FIELDS)
    return Result(len(store), len(dead), starved, example, complete, robot_parts)


def _count_starved(store: _Store, robots: int, edges_from, edges_to) -> int:
    """로봇마다 주행을 원하는 상태에서 주행에 이를 수 있는 상태를 뺀 수."""
    wants = np.zeros(len(store.parts), dtype=bool)
    drives = np.zeros(len(store.parts), dtype=bool)
    for number, part in enumerate(store.parts):
        if len(part) == ROBOT_FIELDS:
            phase, path, idx, _, goal, settle, *_ = part
            drives[number] = goal is not None
            wants[number] = phase.startswith("TO_") and (
                goal is not None or settle is not None or idx < len(path) - 1)
    sources = np.frombuffer(edges_from, dtype=np.uint32)
    targets = np.frombuffer(edges_to, dtype=np.uint32)
    total = 0
    for i in range(robots):
        column = np.frombuffer(store.columns[i], dtype=np.uint32)
        reaches = drives[column].copy()
        while True:
            step = reaches[targets] & ~reaches[sources]
            if not step.any():
                break
            reaches[sources[step]] = True
        total += int((wants[column] & ~reaches).sum())
    return total


def _way_to(dead: list[int], parents, parent_events, event_list) -> list[str]:
    if not dead:
        return []
    way = []
    node = dead[0]
    while node:
        way.append(" ".join(event_list[parent_events[node]]))
        node = parents[node]
    return way[::-1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", maxsplit=1)[0])
    parser.add_argument("--map", default=None,
                        help="lane map YAML (default: the one in fleet_core)")
    parser.add_argument("--robots", type=int, default=3)
    parser.add_argument("--bundle", choices=("file", "on", "off"), default="file",
                        help="two-way bundling: as the map says, or forced")
    parser.add_argument("--start", default=None, help="comma-separated start waypoints")
    parser.add_argument("--max-states", type=int, default=0, help="stop early and say so")
    parser.add_argument("--stop-at-first", action="store_true",
                        help="stop at the first level with a deadlock, for a map too big to finish")
    parser.add_argument("--workers", type=int, default=1, help="processes working on the states")
    parser.add_argument("--confirm", type=int, default=1,
                        help="reports past a zone that free it. The real core takes 3")
    parser.add_argument("--goal-mode", choices=("file", "point", "path"), default="file",
                        help="lane_goal_mode: as fleet_params.yaml says, or forced")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    bundle = {"file": None, "on": True, "off": False}[args.bundle]
    site = load_lane_map(args.map, bundle_two_way=bundle)
    start = args.start.split(",") if args.start else None
    params = load_params()
    if args.goal_mode != "file":
        params = replace(params, lane_goal_mode=args.goal_mode)

    def show(known, frontier, dead):
        print(f"  {known} known, {frontier} to do, {dead} deadlocks", file=sys.stderr, flush=True)

    result = check(site, args.robots, start, params, max_states=args.max_states,
                   workers=args.workers,
                   progress=None if args.quiet else show, stop_at_first=args.stop_at_first,
                   confirm=args.confirm)
    if args.json:
        print(json.dumps(vars(result)))
        return 0
    print(f"({result.states}, {result.deadlocks}, {result.starved})"
          + ("" if result.complete else " stopped early, starved not counted"))
    if result.example:
        print("shortest way into a deadlock:")
        for step in result.example:
            print(f"  {step}")
    return 0


if __name__ == "__main__":
    os.environ.setdefault("PYTHONDONTWRITEBYTECODE", "1")
    sys.exit(main())
