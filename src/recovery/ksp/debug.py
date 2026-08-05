"""Debug drawing tools on a dedicated kRPC connection.

Drawing RPCs run on their own connection so they never contend with the
control loops. Reference frames are re-created on this connection (frames are
per-connection objects) using the same math as the control side.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import krpc

from .reference_frames import create_target_reference_frame

Vec3 = tuple[float, float, float]
RGB = tuple[float, float, float]

_AXIS_COLORS: dict[str, RGB] = {
    "x": (1.0, 0.0, 0.0),
    "y": (0.0, 1.0, 0.0),
    "z": (0.0, 0.0, 1.0),
}


def _points(positions: Sequence[Any]) -> list[Vec3]:
    return [tuple(position) for position in positions]


class DebugConnection:
    def __init__(self, *, name: str, address: str, rpc_port: int, stream_port: int) -> None:
        self._client = krpc.connect(
            name=name, address=address, rpc_port=rpc_port, stream_port=stream_port
        )
        self._closed = False

    @property
    def client(self) -> Any:
        return self._client

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._client.close()


class DebugLine:
    def __init__(self, line: Any) -> None:
        self._line = line

    @property
    def color(self) -> RGB:
        return tuple(self._line.color)

    @color.setter
    def color(self, value: RGB) -> None:
        self._line.color = value

    @property
    def visible(self) -> bool:
        return bool(self._line.visible)

    @visible.setter
    def visible(self, value: bool) -> None:
        self._line.visible = value

    @property
    def thickness(self) -> float:
        return float(self._line.thickness)

    @thickness.setter
    def thickness(self, value: float) -> None:
        self._line.thickness = value

    def set_points(self, start: Vec3, end: Vec3) -> None:
        self._line.start = start
        self._line.end = end

    def remove(self) -> None:
        self._line.remove()

    def clear(self) -> None:
        self.remove()


class DebugMarker:
    def __init__(self, lines: list[DebugLine]) -> None:
        self._lines = lines

    @property
    def visible(self) -> bool:
        return all(line.visible for line in self._lines)

    @visible.setter
    def visible(self, value: bool) -> None:
        for line in self._lines:
            line.visible = value

    def clear(self) -> None:
        for line in self._lines:
            line.remove()
        self._lines = []


class DebugTrajectory:
    def __init__(
        self,
        *,
        client: Any,
        frame: Any,
        name: str,
        positions: Sequence[Any],
        color: RGB,
        thickness: float,
    ) -> None:
        self._client = client
        self._frame = frame
        self.name = name
        self._color = color
        self._thickness = thickness
        self._lines: list[DebugLine] = []
        self.update(positions)

    @property
    def color(self) -> RGB:
        return self._color

    @color.setter
    def color(self, value: RGB) -> None:
        self._color = value
        for line in self._lines:
            line.color = value

    @property
    def visible(self) -> bool:
        return all(line.visible for line in self._lines)

    @visible.setter
    def visible(self, value: bool) -> None:
        for line in self._lines:
            line.visible = value

    @property
    def thickness(self) -> float:
        return self._thickness

    @thickness.setter
    def thickness(self, value: float) -> None:
        self._thickness = value
        for line in self._lines:
            line.thickness = value

    def update(self, positions: Sequence[Any]) -> None:
        points = _points(positions)
        if len(points) - 1 != len(self._lines):
            self._build(points)
            return
        for index, line in enumerate(self._lines):
            line.set_points(points[index], points[index + 1])

    def clear(self) -> None:
        for line in self._lines:
            line.remove()
        self._lines = []

    def _build(self, points: list[Vec3]) -> None:
        self.clear()
        if len(points) < 2:
            return
        drawing = self._client.drawing
        for index in range(len(points) - 1):
            line = drawing.add_line(points[index], points[index + 1], self._frame)
            line.color = self._color
            line.thickness = self._thickness
            self._lines.append(DebugLine(line))


class DebugProxy:
    def __init__(
        self,
        *,
        client: Any,
        body_name: str,
        vessel_name: str,
        target_lon: float | None,
        target_lat: float | None,
    ) -> None:
        self._client = client
        self._body_name = body_name
        self._vessel_name = vessel_name
        self._target_lon = target_lon
        self._target_lat = target_lat
        self._frames: dict[str, Any] = {}
        self._debug_vessel: Any = None
        self._owned: list[Any] = []
        self._trajectories: dict[str, DebugTrajectory] = {}

    def _resolve_debug_vessel(self) -> Any:
        if self._debug_vessel is not None:
            return self._debug_vessel
        for vessel in self._client.space_center.vessels:
            if vessel.name == self._vessel_name:
                self._debug_vessel = vessel
                return vessel
        raise RuntimeError(
            f"no vessel named {self._vessel_name!r} found on debug connection"
        )

    def set_target(self, *, lon: float, lat: float) -> None:
        self._target_lon = lon
        self._target_lat = lat
        self._frames.pop("target", None)

    def _frame(self, frame_name: str) -> Any:
        cached = self._frames.get(frame_name)
        if cached is not None:
            return cached
        space_center = self._client.space_center
        if frame_name == "target":
            if self._target_lon is None or self._target_lat is None:
                raise RuntimeError("no target registered for this booster")
            body = space_center.bodies[self._body_name]
            frame = create_target_reference_frame(
                space_center, body, self._target_lon, self._target_lat
            )
        elif frame_name == "body":
            frame = space_center.bodies[self._body_name].reference_frame
        elif frame_name == "vessel":
            frame = self._resolve_debug_vessel().reference_frame
        elif frame_name == "surface":
            frame = self._resolve_debug_vessel().surface_reference_frame
        elif frame_name == "orbital":
            frame = self._resolve_debug_vessel().orbital_reference_frame
        else:
            raise KeyError(
                f"unknown frame {frame_name!r}"
                f" (available: target, body, vessel, surface, orbital)"
            )
        self._frames[frame_name] = frame
        return frame

    def reference_frame(self, *, frame_name: str = "target", length: float = 10.0) -> DebugMarker:
        frame = self._frame(frame_name)
        origin = (0.0, 0.0, 0.0)
        ends: list[Vec3] = [(length, 0.0, 0.0), (0.0, length, 0.0), (0.0, 0.0, length)]
        lines = [
            DebugLine(self._client.drawing.add_line(origin, end, frame)) for end in ends
        ]
        for line, axis in zip(lines, "xyz", strict=True):
            line.color = _AXIS_COLORS[axis]
            line.thickness = 0.1
        marker = DebugMarker(lines)
        self._owned.append(marker)
        return marker

    def direction(
        self,
        direction: Vec3,
        *,
        frame_name: str = "target",
        length: float = 10.0,
        color: RGB = (1.0, 1.0, 1.0),
        thickness: float = 0.1,
    ) -> DebugLine:
        frame = self._frame(frame_name)
        line = DebugLine(self._client.drawing.add_direction(direction, frame, length=length))
        line.color = color
        line.thickness = thickness
        self._owned.append(line)
        return line

    def line(
        self,
        start: Vec3,
        end: Vec3,
        *,
        frame_name: str = "target",
        color: RGB = (1.0, 1.0, 1.0),
        thickness: float = 0.1,
    ) -> DebugLine:
        frame = self._frame(frame_name)
        line = DebugLine(self._client.drawing.add_line(start, end, frame))
        line.color = color
        line.thickness = thickness
        self._owned.append(line)
        return line

    def trajectory(
        self,
        positions: Sequence[Any] | str | None = None,
        *,
        name: str | None = None,
        frame_name: str = "target",
        color: RGB = (0.0, 1.0, 0.0),
        thickness: float = 0.2,
    ) -> DebugTrajectory | None:
        if isinstance(positions, str):
            return self._trajectories.get(positions)
        if name is None:
            raise ValueError("name is required when creating a trajectory")
        if positions is None:
            raise ValueError("positions are required when creating a trajectory")
        trajectory = self._trajectories.get(name)
        if trajectory is None:
            trajectory = DebugTrajectory(
                client=self._client,
                frame=self._frame(frame_name),
                name=name,
                positions=positions,
                color=color,
                thickness=thickness,
            )
            self._trajectories[name] = trajectory
            self._owned.append(trajectory)
        else:
            trajectory.update(positions)
        return trajectory

    @property
    def trajectories(self) -> dict[str, DebugTrajectory]:
        return dict(self._trajectories)

    def clear(self, name: str) -> None:
        trajectory = self._trajectories.pop(name, None)
        if trajectory is not None:
            trajectory.clear()
            if trajectory in self._owned:
                self._owned.remove(trajectory)

    def clear_all(self) -> None:
        for drawable in self._owned:
            drawable.clear()
        self._owned = []
        self._trajectories = {}
