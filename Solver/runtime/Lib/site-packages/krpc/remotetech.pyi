from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.spacecenter import SpaceCenter


class RemoteTech:

    @property
    def available(self):
        # type: () -> bool
        """Whether RemoteTech is installed."""
        ...

    @property
    def ground_stations(self):
        # type: () -> List[str]
        """The names of the ground stations."""
        ...

    def antenna(self, part):
        # type: (SpaceCenter.Part) -> RemoteTech.Antenna
        """Get the antenna object for a particular part."""
        ...

    def comms(self, vessel):
        # type: (SpaceCenter.Vessel) -> RemoteTech.Comms
        """Get a communications object, representing the communication capability of a particular vessel."""
        ...

    class Antenna:
        @property
        def has_connection(self):
            # type: () -> bool
            """Whether the antenna has a connection."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """Get the part containing this antenna."""
            ...

        @property
        def target(self):
            # type: () -> RemoteTech.Target
            """The object that the antenna is targetting.
            This property can be used to set the target to RemoteTech.Target#none or RemoteTech.Target#activeVessel.
            To set the target to a celestial body, ground station or vessel see RemoteTech.Antenna#targetBody,
            RemoteTech.Antenna#targetGroundStation and RemoteTech.Antenna#targetVessel."""
            ...

        @target.setter
        def target(self, value):
            # type: (RemoteTech.Target) -> None
            ...

        @property
        def target_body(self):
            # type: () -> SpaceCenter.CelestialBody
            """The celestial body the antenna is targetting."""
            ...

        @target_body.setter
        def target_body(self, value):
            # type: (SpaceCenter.CelestialBody) -> None
            ...

        @property
        def target_ground_station(self):
            # type: () -> str
            """The ground station the antenna is targetting."""
            ...

        @target_ground_station.setter
        def target_ground_station(self, value):
            # type: (str) -> None
            ...

        @property
        def target_vessel(self):
            # type: () -> SpaceCenter.Vessel
            """The vessel the antenna is targetting."""
            ...

        @target_vessel.setter
        def target_vessel(self, value):
            # type: (SpaceCenter.Vessel) -> None
            ...

    class Comms:
        @property
        def antennas(self):
            # type: () -> List[RemoteTech.Antenna]
            """The antennas for this vessel."""
            ...

        @property
        def has_connection(self):
            # type: () -> bool
            """Whether the vessel has any connection."""
            ...

        @property
        def has_connection_to_ground_station(self):
            # type: () -> bool
            """Whether the vessel has a connection to a ground station."""
            ...

        @property
        def has_flight_computer(self):
            # type: () -> bool
            """Whether the vessel has a flight computer on board."""
            ...

        @property
        def has_local_control(self):
            # type: () -> bool
            """Whether the vessel can be controlled locally."""
            ...

        @property
        def signal_delay(self):
            # type: () -> float
            """The shortest signal delay to the vessel, in seconds."""
            ...

        @property
        def signal_delay_to_ground_station(self):
            # type: () -> float
            """The signal delay between the vessel and the closest ground station, in seconds."""
            ...

        @property
        def vessel(self):
            # type: () -> SpaceCenter.Vessel
            """Get the vessel."""
            ...

        def signal_delay_to_vessel(self, other):
            # type: (SpaceCenter.Vessel) -> float
            """The signal delay between the this vessel and another vessel, in seconds.
            :other """
            ...

    class Target:
        """The type of object an antenna is targetting.
        See RemoteTech.Antenna#target."""
        active_vessel = 0
        celestial_body = 1
        ground_station = 2
        vessel = 3
        none__ = 4
