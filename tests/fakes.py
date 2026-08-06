"""Minimal kRPC client fakes for unit tests (no live server required)."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

Vector = tuple[float, float, float]


class FakeAutoPilot:
    def __init__(self) -> None:
        self.engaged = False
        self.reference_frame: Any = None
        self.target_direction: Vector | None = None
        self.up_reference: Vector | None = None
        self.target_roll: float | None = None
        self.target_smoothing_time = 0.0


class FakeControl:
    def __init__(self) -> None:
        self.throttle = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.sas = False
        self.rcs = False
        self.legs = False
        self.gear = False
        self.lights = False
        self.brakes = False
        self.abort = False
        self.stage_calls = 0

    def activate_next_stage(self) -> list[Any]:
        self.stage_calls += 1
        return []

    def set_action_group(self, group: int, state: bool) -> None:
        self._group = (group, state)

    def get_action_group(self, group: int) -> bool:
        return bool(getattr(self, "_group", (group, False))[1])

    def toggle_action_group(self, group: int) -> None:
        self._group = (group, not self.get_action_group(group))


class FakeEngine:
    def __init__(self, thrust: float, isp: float, *, active: bool = True) -> None:
        self.thrust = thrust
        self.specific_impulse = isp
        self.active = active


class FakeParts:
    def __init__(self, engines: list[FakeEngine]) -> None:
        self.engines = engines


class FakeFlight:
    mean_altitude = 5000.0
    surface_altitude = 300.0
    atmosphere_density = 0.1


class FakeBody:
    def __init__(
        self,
        *,
        name: str = "Kerbin",
        equatorial_radius: float = 600000.0,
        bedrock: float = -100.0,
        surface: float = 50.0,
    ) -> None:
        self.name = name
        self.equatorial_radius = equatorial_radius
        self.reference_frame = object()
        self._bedrock = bedrock
        self._surface = surface

    def bedrock_height(self, lat: float, lon: float) -> float:
        return self._bedrock

    def surface_height(self, lat: float, lon: float) -> float:
        return self._surface


class FakeOrbit:
    def __init__(self, body: FakeBody) -> None:
        self.body = body


class FakeVessel:
    def __init__(self, name: str = "Booster 1", *, engines: list[FakeEngine] | None = None) -> None:
        self.name = name
        self.mass = 30000.0
        self.dry_mass = 10000.0
        self.thrust = 100000.0
        self.available_thrust = 300000.0
        self.max_thrust = 400000.0
        self.max_vacuum_thrust = 450000.0
        self.met = 100.0
        self.loaded = True
        self.packed = False
        self.physics_range = 0.0
        self.surface_reference_frame = object()
        self.orbital_reference_frame = object()
        self.reference_frame = object()
        self.control = FakeControl()
        self.auto_pilot = FakeAutoPilot()
        self.parts = FakeParts(engines if engines is not None else [FakeEngine(100000.0, 300.0)])
        self.orbit = FakeOrbit(FakeBody())
        self.situation = SimpleNamespace(name="flying")

    def position(self, frame: Any) -> Vector:
        return (10.0, 20.0, 30.0)

    def velocity(self, frame: Any) -> Vector:
        return (1.0, 2.0, 3.0)

    def rotation(self, frame: Any) -> tuple[float, float, float, float]:
        return (0.0, 0.0, 0.0, 1.0)

    def angular_velocity(self, frame: Any) -> Vector:
        return (0.1, -0.2, 0.05)

    def flight(self, frame: Any) -> FakeFlight:
        return FakeFlight()


class FakeReferenceFrame:
    calls: list[tuple[Vector, tuple[float, float, float, float]]] = []

    @classmethod
    def create_relative(
        cls,
        reference_frame: Any,
        position: Vector = (0.0, 0.0, 0.0),
        rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        velocity: Vector = (0.0, 0.0, 0.0),
        angular_velocity: Vector = (0.0, 0.0, 0.0),
    ) -> str:
        cls.calls.append((position, rotation))
        return f"frame-{len(cls.calls)}"


class FakeSpaceCenter:
    def __init__(self, vessels: list[FakeVessel], *, ut: float = 1000.0) -> None:
        self.vessels = vessels
        self.ut = ut
        self.ReferenceFrame = FakeReferenceFrame
        self.bodies: dict[str, FakeBody] = {}
        for vessel in vessels:
            body = vessel.orbit.body
            self.bodies[body.name] = body


class FakeStream:
    def __init__(self, getter: Callable[[], Any]) -> None:
        self._getter = getter
        self.started = False

    def start(self) -> None:
        self.started = True

    def remove(self) -> None:
        self.started = False

    def __call__(self) -> Any:
        return self._getter()


class FakeLine:
    def __init__(self, start: Vector, end: Vector, frame: Any) -> None:
        self.start = start
        self.end = end
        self.frame = frame
        self.visible = True
        self.color: Vector = (1.0, 1.0, 1.0)
        self.thickness = 0.1
        self.removed = False

    def remove(self) -> None:
        self.removed = True


class FakeDrawing:
    def __init__(self) -> None:
        self.lines: list[FakeLine] = []

    def add_line(self, start: Vector, end: Vector, frame: Any, *, visible: bool = True) -> FakeLine:
        line = FakeLine(start, end, frame)
        line.visible = visible
        self.lines.append(line)
        return line

    def add_direction(
        self, direction: Vector, frame: Any, *, length: float = 10.0, visible: bool = True
    ) -> FakeLine:
        line = FakeLine((0.0, 0.0, 0.0), direction, frame)
        line.visible = visible
        self.lines.append(line)
        return line


class FakeClient:
    def __init__(self, vessels: list[FakeVessel], *, ut: float = 1000.0) -> None:
        self.space_center = FakeSpaceCenter(vessels, ut=ut)
        self.drawing = FakeDrawing()
        self.stream_update_condition = threading.Condition()
        self.streams: list[FakeStream] = []
        self.closed = False
        self._pending = threading.Event()

    def add_stream(self, func: Callable[..., Any], *args: Any) -> FakeStream:
        stream = FakeStream(lambda: func(*args))
        self.streams.append(stream)
        return stream

    def wait_for_stream_update(self, timeout: float | None = None) -> bool:
        if self._pending.is_set():
            self._pending.clear()
            return True
        time.sleep(min(timeout or 0.01, 0.01))
        return False

    def close(self) -> None:
        self.closed = True
