"""Goal handles and pending reports of the wrapper. Plain Python, so it is easy to test."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class _Cancel:
    tried_at: float
    tries: int = 1
    warned: bool = False


class GoalTracker:
    """A cancel counts as done when the result comes, and until then it must not be lost: not
    while the server has not answered the goal yet, and not when it is lost or refused."""

    def __init__(self):
        self._pending: set = set()
        self._handles: dict = {}
        self._cancels: dict = {}

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

    def cancel(self, key, now: float) -> None:
        if key in self._cancels:
            return
        handle = self._handles.get(key)
        if handle is not None:
            handle.cancel_goal_async()
        elif key not in self._pending:
            return
        self._cancels[key] = _Cancel(now)

    def retry_cancels(self, now: float, retry_s: float, warn_after: int) -> list:
        """Ask again for what is due. Returns the keys that have just used up warn_after tries,
        each once. Time waiting for an acceptance counts: the goal may be running all the same."""
        unconfirmed = []
        for key, cancel in list(self._cancels.items()):
            if now - cancel.tried_at < retry_s:
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
    """Keeps order reports until central has answered them. Reports are told apart by a number:
    what is in them can be the same twice, and holds dicts."""

    def __init__(self):
        self._items: dict = {}
        self._next_id = 0

    def add(self, item) -> None:
        self._items[self._next_id] = _Report(item)
        self._next_id += 1

    def due(self, now: float, ready: bool, retry_s: float) -> list:
        """(number, report) of what to send now: not sent yet, or no answer for retry_s."""
        if not ready:
            return []
        out = []
        for report_id, report in list(self._items.items()):
            if report.sent_at is None or now - report.sent_at >= retry_s:
                report.sent_at = now
                out.append((report_id, report.item))
        return out

    def done(self, report_id) -> bool:
        """Drop an answered report. False if an earlier answer already did."""
        return self._items.pop(report_id, None) is not None
