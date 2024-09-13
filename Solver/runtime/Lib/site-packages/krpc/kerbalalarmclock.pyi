from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.spacecenter import SpaceCenter


class KerbalAlarmClock:

    @property
    def alarms(self):
        # type: () -> List[KerbalAlarmClock.Alarm]
        """A list of all the alarms."""
        ...

    @property
    def available(self):
        # type: () -> bool
        """Whether Kerbal Alarm Clock is available."""
        ...

    def alarm_with_name(self, name):
        # type: (str) -> Optional[KerbalAlarmClock.Alarm]
        """Get the alarm with the given name, or {@code null}
        if no alarms have that name. If more than one alarm has the name,
        only returns one of them.
        :name Name of the alarm to search for."""
        ...

    def alarms_with_type(self, type):
        # type: (KerbalAlarmClock.AlarmType) -> List[KerbalAlarmClock.Alarm]
        """Get a list of alarms of the specified type.
        :type Type of alarm to return."""
        ...

    def create_alarm(self, type, name, ut):
        # type: (KerbalAlarmClock.AlarmType, str, float) -> KerbalAlarmClock.Alarm
        """Create a new alarm and return it.
        :type Type of the new alarm.
        :name Name of the new alarm.
        :ut Time at which the new alarm should trigger."""
        ...

    class Alarm:
        @property
        def action(self):
            # type: () -> KerbalAlarmClock.AlarmAction
            """The action that the alarm triggers."""
            ...

        @action.setter
        def action(self, value):
            # type: (KerbalAlarmClock.AlarmAction) -> None
            ...

        @property
        def id(self):
            # type: () -> str
            """The unique identifier for the alarm."""
            ...

        @property
        def margin(self):
            # type: () -> float
            """The number of seconds before the event that the alarm will fire."""
            ...

        @margin.setter
        def margin(self, value):
            # type: (float) -> None
            ...

        @property
        def name(self):
            # type: () -> str
            """The short name of the alarm."""
            ...

        @name.setter
        def name(self, value):
            # type: (str) -> None
            ...

        @property
        def notes(self):
            # type: () -> str
            """The long description of the alarm."""
            ...

        @notes.setter
        def notes(self, value):
            # type: (str) -> None
            ...

        @property
        def remaining(self):
            # type: () -> float
            """The number of seconds until the alarm will fire."""
            ...

        @property
        def repeat(self):
            # type: () -> bool
            """Whether the alarm will be repeated after it has fired."""
            ...

        @repeat.setter
        def repeat(self, value):
            # type: (bool) -> None
            ...

        @property
        def repeat_period(self):
            # type: () -> float
            """The time delay to automatically create an alarm after it has fired."""
            ...

        @repeat_period.setter
        def repeat_period(self, value):
            # type: (float) -> None
            ...

        @property
        def time(self):
            # type: () -> float
            """The time at which the alarm will fire."""
            ...

        @time.setter
        def time(self, value):
            # type: (float) -> None
            ...

        @property
        def type(self):
            # type: () -> KerbalAlarmClock.AlarmType
            """The type of the alarm."""
            ...

        @property
        def vessel(self):
            # type: () -> SpaceCenter.Vessel
            """The vessel that the alarm is attached to."""
            ...

        @vessel.setter
        def vessel(self, value):
            # type: (SpaceCenter.Vessel) -> None
            ...

        @property
        def xfer_origin_body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body the vessel is departing from."""
            ...

        @xfer_origin_body.setter
        def xfer_origin_body(self, value):
            # type: (SpaceCenter.CelestialBody) -> None
            ...

        @property
        def xfer_target_body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body the vessel is arriving at."""
            ...

        @xfer_target_body.setter
        def xfer_target_body(self, value):
            # type: (SpaceCenter.CelestialBody) -> None
            ...

        def remove(self):
            # type: () -> None
            """Removes the alarm."""
            ...

    class AlarmAction:
        """The action performed by an alarm when it fires."""
        do_nothing = 0
        do_nothing_delete_when_passed = 1
        kill_warp = 2
        kill_warp_only = 3
        message_only = 4
        pause_game = 5

    class AlarmType:
        """The type of an alarm."""
        raw = 0
        maneuver = 1
        maneuver_auto = 2
        apoapsis = 3
        periapsis = 4
        ascending_node = 5
        descending_node = 6
        closest = 7
        contract = 8
        contract_auto = 9
        crew = 10
        distance = 11
        earth_time = 12
        launch_rendevous = 13
        soi_change = 14
        soi_change_auto = 15
        transfer = 16
        transfer_modelled = 17
