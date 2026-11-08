"""래퍼의 goal 핸들과 대기 중인 보고. 순수 파이썬이라 테스트하기 쉽다."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class _Cancel:
    tried_at: float
    tries: int = 1
    warned: bool = False


class GoalTracker:
    """취소는 결과가 와야 끝난 것이다. 그때까지 잃으면 안 된다. 서버가 goal에 아직
    답하지 않았을 때도, goal이 유실되거나 거절됐을 때도 마찬가지다."""

    def __init__(self):
        self._pending: set = set()
        self._handles: dict = {}
        self._cancels: dict = {}
        self._lost: set = set()  # 결과를 잃은 취소. 경고한 뒤 그만둔다
        self._dropped: list = []  # 그만둔 잃은 취소. 코어에 알릴 때까지 둔다

    def sent(self, key) -> None:
        self._pending.add(key)

    def accepted(self, key, handle) -> None:
        self._pending.discard(key)
        self._handles[key] = handle
        if key in self._cancels:
            handle.cancel_goal_async()

    def finished(self, key) -> None:
        self._pending.discard(key)
        self._handles.pop(key, None)
        self._cancels.pop(key, None)
        self._lost.discard(key)

    def lost(self, key) -> None:
        """취소한 goal의 결과를 잃었다. 결과는 다시 오지 않으니 경고까지만 취소를 다시 보낸다."""
        if key in self._cancels:
            self._lost.add(key)

    def cancel(self, key, now: float) -> None:
        if key in self._cancels:
            return
        handle = self._handles.get(key)
        if handle is not None:
            handle.cancel_goal_async()
        elif key not in self._pending:
            return
        self._cancels[key] = _Cancel(now)

    def canceling(self, key) -> bool:
        """취소를 보냈고 결과는 아직이다."""
        return key in self._cancels

    def take_dropped(self) -> list:
        """결과를 잃고 그만둔 취소의 key. 한 번씩만 돌려준다."""
        dropped, self._dropped = self._dropped, []
        return dropped

    def retry_cancels(self, now: float, retry_s: float, warn_after: int) -> list:
        """때가 된 취소를 다시 요청한다. warn_after번을 막 채운 key를 한 번씩 돌려준다.
        수락을 기다리는 시간도 센다. goal은 그동안 돌고 있을 수 있다."""
        unconfirmed = []
        for key, cancel in list(self._cancels.items()):
            if now - cancel.tried_at < retry_s:
                continue
            if cancel.warned and key in self._lost:
                self.finished(key)  # 잃은 결과는 다시 오지 않는다
                self._dropped.append(key)
                continue
            if cancel.tries >= warn_after and not cancel.warned:
                cancel.warned = True
                unconfirmed.append(key)
            cancel.tried_at = now
            cancel.tries += 1
            handle = self._handles.get(key)
            if handle is not None:
                handle.cancel_goal_async()
        return unconfirmed


@dataclass
class _Report:
    item: object
    sent_at: float | None = None


class ReportOutbox:
    """central이 답할 때까지 주문 보고를 보관한다. 보고는 번호로 구분한다.
    내용이 같을 수 있고 dict를 담고 있기 때문이다."""

    def __init__(self):
        self._items: dict = {}
        self._next_id = 0

    def add(self, item) -> None:
        self._items[self._next_id] = _Report(item)
        self._next_id += 1

    def due(self, now: float, ready: bool, retry_s: float) -> list:
        """지금 보낼 (번호, 보고). 아직 안 보냈거나 retry_s 동안 답이 없는 것."""
        if not ready:
            return []
        out = []
        for report_id, report in list(self._items.items()):
            if report.sent_at is None or now - report.sent_at >= retry_s:
                report.sent_at = now
                out.append((report_id, report.item))
        return out

    def done(self, report_id) -> bool:
        """답을 받은 보고를 지운다. 앞선 답이 이미 지웠으면 False."""
        return self._items.pop(report_id, None) is not None
