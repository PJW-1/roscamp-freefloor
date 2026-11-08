"""Nav2 behavior_server의 backup(nav2_msgs/action/BackUp) 액션 클라이언트.

backup.BackUp 명령을 Nav2 목표로 보내고, 결과를 (순번, outcome, error_code, 물러난 거리)로
돌려준다. 결과의 뜻은 backup.reason_of가 정한다.

쓰는 곳
- 지금: pinky_fsm 노드의 자가 복구.
- 나중: 버전 2 Retreat 액션 서버. execute에서 send를 부르고 on_result로 Retreat 결과
  (success, reason=reason_of(...))를 채운다. 취소는 cancel_all이다.

on_result는 이 클래스의 잠금을 쥐지 않은 채 부른다. 부르는 쪽은 자기 잠금을 쥐고 send나
cancel_all을 불러도 된다.
"""
from __future__ import annotations

import threading
from collections.abc import Callable
from functools import partial

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp as BackUpAction
from rclpy.action import ActionClient

_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled"}

# (순번, outcome, error_code, 마지막 feedback의 물러난 거리 m)
OnResult = Callable[[int, str, int, float], None]


class BackUpRunner:
    def __init__(self, node, action_name: str, on_result: OnResult, callback_group=None):
        self._client = ActionClient(node, BackUpAction, action_name,
                                    callback_group=callback_group)
        self._on_result = on_result
        self._lock = threading.Lock()
        self._handles: dict[int, object] = {}  # 순번 → 수락된 목표
        self._pending: set[int] = set()  # 보냈지만 수락 여부를 아직 모른다
        self._cancel_on_accept: set[int] = set()
        self._travelled: dict[int, float] = {}

    def send(self, seq: int, distance_m: float, speed_mps: float,
             time_allowance_s: float) -> bool:
        """보냈으면 True. 서버가 없으면 False이고 결과 콜백도 오지 않는다."""
        if not self._client.server_is_ready():
            return False
        sec = int(time_allowance_s)
        goal = BackUpAction.Goal(
            target=Point(x=float(distance_m)), speed=float(speed_mps),
            time_allowance=Duration(sec=sec, nanosec=int((time_allowance_s - sec) * 1e9)))
        with self._lock:
            self._pending.add(seq)
            self._travelled[seq] = 0.0
        future = self._client.send_goal_async(goal, feedback_callback=partial(self._feedback, seq))
        future.add_done_callback(partial(self._on_response, seq))
        return True

    def cancel_all(self) -> None:
        with self._lock:
            handles = list(self._handles.values())
            self._cancel_on_accept |= self._pending
        for handle in handles:
            handle.cancel_goal_async()

    def destroy(self) -> None:
        self._client.destroy()

    def _feedback(self, seq: int, msg) -> None:
        with self._lock:
            self._travelled[seq] = float(msg.feedback.distance_traveled)

    def _on_response(self, seq: int, future) -> None:
        handle = future.result()
        with self._lock:
            self._pending.discard(seq)
            if not handle.accepted:
                travelled = self._travelled.pop(seq, 0.0)
            else:
                self._handles[seq] = handle
                cancel = seq in self._cancel_on_accept
                self._cancel_on_accept.discard(seq)
        if not handle.accepted:
            self._on_result(seq, "rejected", 0, travelled)
            return
        if cancel:
            handle.cancel_goal_async()  # 멈추는 동안 수락된 목표는 바로 취소한다
        handle.get_result_async().add_done_callback(partial(self._on_done, seq))

    def _on_done(self, seq: int, future) -> None:
        res = future.result()
        with self._lock:
            self._handles.pop(seq, None)
            travelled = self._travelled.pop(seq, 0.0)
        self._on_result(seq, _OUTCOME.get(res.status, "aborted"), int(res.result.error_code),
                        travelled)
