#!/usr/bin/env python3
"""구역 해제 규칙 압박 시험: 보고 지연과 위치 잡음 아래 시나리오를 돌리고 표를 출력한다.

    python3 tools/pressure_sweep.py --out /some/dir
    python3 tools/pressure_sweep.py --out /some/dir --d 0.2 0.1 --tau 0 0.5 --eps 0 0.05

tau는 report_delay_s(시나리오 초), eps는 pose_noise_m(축당 1 시그마)다. 각 행은 가짜 로봇으로
`ros2 launch iponic_bringup pc.launch.py`를 실제로 돌린 결과라 워크스페이스를 빌드하고 source해야
한다. 실행마다 --first-domain부터 따로 도메인을 쓰고, 로봇 네트워크에는 닿지 않으며(LOCALHOST),
파일은 --out 아래에 둔다.

열의 뜻:
    early, near     실제 위치에 대한 SafetyMeter. early: 몸체가 구역에서 반지름 1 이내인데
                    해제됨. near: 두 몸체가 2r보다 가까움.
    deadlocks       DEADLOCK_DETECTED 이벤트 수.
    lane errors     LANE_ERROR 이벤트 수: lane_error_warn_m보다 더 벗어난 보고.
    needs d         r + eps + v * (tau + tick). fleet_params.yaml의 규칙이며 v는 시나리오의
                    최고 속도다. 이 값이 그 행의 d보다 크면 규칙은 그 실행을 보장하지 않는다.
    pass            시나리오 자체의 판정. 압박 아래에서는 기대 이벤트 수가 어긋날 수 있다.
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import os
import signal
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "pc" / "fleet_core"))

from fleet_core.common.params import DEFAULT_PARAMS  # noqa: E402

SCENARIOS = REPO / "sim" / "fake_robots" / "scenarios"
ROBOT_NETWORK_DOMAIN = 6
RADIUS_M = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))["robot_radius_m"]
TOP_SPEED_MPS = 0.2
TICK_S = 0.1
RESULT_FILES = ("summary.json", "safety.json", "events.csv")


@dataclass(frozen=True)
class Case:
    scenario: str
    tau: float
    eps: float
    d: float
    stop: float = 0.0

    @property
    def name(self) -> str:
        return f"{self.scenario}_t{self.tau}_e{self.eps}_d{self.d}_s{self.stop}"


@dataclass(frozen=True)
class Row:
    case: Case
    finished: bool
    passed: bool = False
    shipped: int = 0
    total: int = 0
    seconds: float = 0.0
    early: int = 0
    near: int = 0
    deadlocks: int = 0
    lane_errors: int = 0
    releases: int = 0
    min_release_m: float = 0.0
    min_gap_m: float = 0.0
    failed_checks: tuple = ()
    violations: tuple = ()


def required_distance(tau: float, eps: float) -> float:
    return RADIUS_M + eps + TOP_SPEED_MPS * (tau + TICK_S)


def cases(scenarios, taus, epss, ds, stop: float = 0.0) -> list[Case]:
    if min(ds) <= 2 * RADIUS_M:
        raise ValueError(
            f"d has to exceed 2r = {2 * RADIUS_M} m, or TrafficController refuses to start")
    wanted = (Case(s, t, e, d, stop) for s in scenarios for d in ds for t in taus for e in epss)
    return list(dict.fromkeys(wanted))  # 같은 케이스가 둘이면 폴더 하나에 같이 쓰게 된다


def variant(data: dict, tau: float, eps: float, stop: float = 0.0) -> dict:
    out = copy.deepcopy(data)
    for robot in out["robots"].values():
        robot.update(report_delay_s=tau, pose_noise_m=eps, stop_error_m=stop)
    return out


def write_params(path: Path, d: float) -> Path:
    params = yaml.safe_load(DEFAULT_PARAMS.read_text(encoding="utf-8"))
    params["release_distance_m"] = d
    path.write_text(yaml.safe_dump(params), encoding="utf-8")
    return path


def ros_env(domain: int, base=None) -> dict:
    if domain == ROBOT_NETWORK_DOMAIN:
        raise ValueError(f"domain {domain} is the robot network")
    env = {**(os.environ if base is None else base), "ROS_DOMAIN_ID": str(domain),
           "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    env.pop("ROS_STATIC_PEERS", None)  # LOCALHOST여도 이 값은 닿는다
    return env


def read_row(case: Case, folder: Path) -> Row:
    if not (folder / "summary.json").exists() or not (folder / "safety.json").exists():
        return Row(case, finished=False)
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    safety = json.loads((folder / "safety.json").read_text(encoding="utf-8"))
    with (folder / "events.csv").open(encoding="utf-8") as f:
        codes = [row["code"] for row in csv.DictReader(f)]
    return Row(case, True, summary["pass"], summary["orders_shipped"], summary["orders_total"],
               summary["duration_s"], safety["early"], safety["near"],
               codes.count("DEADLOCK_DETECTED"), codes.count("LANE_ERROR"),
               safety["releases"], safety["min_release_m"] or 0.0, safety["min_gap_m"] or 0.0,
               tuple(k for k, v in summary["checks"].items() if not v["ok"]),
               tuple(summary["violations"]))


def table(rows: list[Row]) -> str:
    lines = ["| scenario | tau | eps | d | needs d | early | near | deadlocks | lane errors "
             "| releases | min release | min gap | shipped | pass |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for row in rows:
        c = row.case
        head = (f"| {c.scenario} | {c.tau} | {c.eps} | {c.d} "
                f"| {required_distance(c.tau, c.eps):.3f} ")
        if not row.finished:
            lines.append(head + "| did not finish | | | | | | | | |")
            continue
        verdict = "PASS" if row.passed else "FAIL " + ",".join(row.failed_checks)
        lines.append(head + f"| {row.early} | {row.near} | {row.deadlocks} | {row.lane_errors} "
                     f"| {row.releases} | {row.min_release_m:.3f} | {row.min_gap_m:.3f} "
                     f"| {row.shipped}/{row.total} | {verdict} |")
    return "\n".join(lines)


def run_case(case: Case, domain: int, out: Path, wall_s: float) -> Row:
    folder = out / case.name
    folder.mkdir(parents=True, exist_ok=True)
    for name in RESULT_FILES:  # 죽은 실행이 지난 실행의 결과를 보이면 안 된다
        (folder / name).unlink(missing_ok=True)
    data = yaml.safe_load((SCENARIOS / f"{case.scenario}.yaml").read_text(encoding="utf-8"))
    scenario = folder / "scenario.yaml"
    scenario.write_text(yaml.safe_dump(variant(data, case.tau, case.eps, case.stop)),
                        encoding="utf-8")
    command = ["ros2", "launch", "iponic_bringup", "pc.launch.py", "use_fake:=true",
               f"scenario:={scenario}", f"result_dir:={folder}",
               f"params_file:={write_params(folder / 'params.yaml', case.d)}"]
    with (folder / "launch.log").open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(command, env=ros_env(domain), stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            launch.wait(timeout=wall_s)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            return Row(case, finished=False)  # 멈추라고 해도 파일은 쓸 수 있다
    return read_row(case, folder)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="+", default=["basic_orders", "order_burst"])
    parser.add_argument("--tau", nargs="+", type=float, default=[0.0, 0.2, 0.5])
    parser.add_argument("--eps", nargs="+", type=float, default=[0.0, 0.03, 0.05])
    parser.add_argument("--d", nargs="+", type=float, default=[0.2])
    parser.add_argument("--stop-error", type=float, default=0.0)
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--first-domain", type=int, default=80)
    parser.add_argument("--wall", type=float, default=480.0, help="seconds before a run is cut")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    wanted = cases(args.scenarios, args.tau, args.eps, args.d, args.stop_error)

    def one(item):
        number, case = item
        row = run_case(case, args.first_domain + number % args.jobs, args.out, args.wall)
        result = (f"early {row.early} near {row.near} pass {row.passed}" if row.finished
                  else "did not finish")
        print(f"{case.name}: {result}", flush=True)
        return row

    # 슬롯 하나의 실행은 차례로 돌아 두 실행이 도메인을 같이 쓰지 않는다.
    slots = [list(enumerate(wanted))[i::args.jobs] for i in range(args.jobs)]
    with ThreadPoolExecutor(args.jobs) as pool:
        done = pool.map(lambda slot: [one(item) for item in slot], slots)
        rows = {row.case: row for slot in done for row in slot}
    text = table([rows[case] for case in wanted])
    (args.out / "table.md").write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
