"""The old one-class FleetCore's private names, mapped onto FleetManager.

Only for the older tests, which force and read fields of what used to be one mission record.
Keeping them unchanged is what shows the split kept the behaviour. New tests use the cores.
"""
from gg_fleet_core.manager import FleetManager

_ORCHESTRATOR_FIELDS = {"basket", "order", "arm", "arm_retry_at", "last_order_id"}


class _Mission:
    """One robot's old record: the fields now live in one core or the other."""

    def __init__(self, core, name):
        object.__setattr__(self, "_core", core)
        object.__setattr__(self, "name", name)

    def _home(self, field):
        if field in _ORCHESTRATOR_FIELDS:
            return self._core.orchestrator._robots[self.name]
        return self._core.traffic._missions[self.name]

    def __getattr__(self, field):
        return getattr(self._home(field), field)

    def __setattr__(self, field, value):
        setattr(self._home(field), field, value)
        if field == "basket" and value == "FULL":
            # A full basket used to mean heading out by itself. Now somebody has to say so.
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
        return self.orchestrator._cleared_jobs

    def _pending(self):
        return self.orchestrator._pending()

    def _fail(self, mission):
        self.traffic._fail(self.traffic._missions[mission.name])

    def _arm_goal_allowed(self, mission, server):
        return self.orchestrator._arm_goal_allowed(
            self.orchestrator._robots[mission.name], server)
