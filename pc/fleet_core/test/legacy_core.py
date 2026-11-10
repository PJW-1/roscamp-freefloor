"""옛 단일 FleetCore의 비공개 이름을 FleetManager에 대응시킨다.

예전 테스트가 필드를 직접 바꾸고 읽을 때만 쓴다. 분리 후에도 동작이 같음을
보이려고 그대로 둔다. 새 테스트는 각 코어를 쓴다.
"""
from fleet_core.manager import FleetManager

_ORCHESTRATOR_FIELDS = {"basket", "order", "last_order_id"}
_ARM_FIELDS = {"arm": "job", "arm_retry_at": "retry_at"}  # old name -> the arm core's name


class _Mission:
    """로봇 하나의 옛 기록. 필드는 이제 두 코어에 나뉘어 있다."""

    def __init__(self, core, name):
        object.__setattr__(self, "_core", core)
        object.__setattr__(self, "name", name)

    def _home(self, field):
        if field in _ORCHESTRATOR_FIELDS:
            return self._core.orchestrator._robots[self.name]
        return self._core.traffic._missions[self.name]

    def __getattr__(self, field):
        if field in _ARM_FIELDS:
            return getattr(self._core.arm._robots[self.name], _ARM_FIELDS[field])
        return getattr(self._home(field), field)

    def __setattr__(self, field, value):
        if field in _ARM_FIELDS:
            setattr(self._core.arm._robots[self.name], _ARM_FIELDS[field], value)
            return
        setattr(self._home(field), field, value)
        if field == "basket" and value == "FULL":
            # 예전에는 바구니가 차면 스스로 나갔다. 지금은 누가 지시해야 한다.
            self._core.orchestrator._send_to_outbound(self._home(field))
            self._core._pump()


class FleetCore(FleetManager):
    @property
    def _missions(self):
        return {name: _Mission(self, name) for name in self.traffic.robots}

    @property
    def _retreating(self):
        return self.traffic._retreating

    @property
    def _cleared_jobs(self):
        return self.arm._cleared_jobs

    def _pending(self):
        return self.orchestrator._pending()

    def _fail(self, mission):
        self.traffic._fail(self.traffic._missions[mission.name])

    def _arm_goal_allowed(self, mission, server):
        return self.arm._goal_allowed(self.arm._robots[mission.name], server)
