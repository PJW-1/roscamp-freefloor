"""Goal handles and pending reports of the wrapper. Plain Python, so it is easy to test."""
from __future__ import annotations


class GoalTracker:
    """A cancel can arrive while the server has not answered yet. It must not be lost."""

    def __init__(self):
        self._pending: set = set()
        self._handles: dict = {}
        self._cancel_wanted: set = set()

    def sent(self, key) -> None:
        self._pending.add(key)

    def accepted(self, key, handle) -> None:
        self._pending.discard(key)
        self._handles[key] = handle
        if key in self._cancel_wanted:
            self._cancel_wanted.discard(key)
            handle.cancel_goal_async()

    def finished(self, key) -> None:
        self._pending.discard(key)
        self._handles.pop(key, None)
        self._cancel_wanted.discard(key)

    def cancel(self, key) -> None:
        handle = self._handles.get(key)
        if handle is not None:
            handle.cancel_goal_async()
        elif key in self._pending:
            self._cancel_wanted.add(key)


class ReportOutbox:
    """Keeps order reports until the central service can take them."""

    def __init__(self):
        self._items: list = []

    def add(self, item) -> None:
        self._items.append(item)

    def drain(self, ready: bool) -> list:
        if not ready:
            return []
        items, self._items = self._items, []
        return items
