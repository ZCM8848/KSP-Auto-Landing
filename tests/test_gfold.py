"""Tests for the pure G-FOLD planner module."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from recovery.guidance.gfold import (
    G0,
    GfoldParams,
    build_config,
    command,
    command_at_time,
    replan,
    solve,
    solve_optimal,
    tof_of,
)
from recovery.types import FlightState, Quaternion, Situation, Vector3

G_SURF = 9.81


def _state(**overrides: object) -> FlightState:
    data: dict[str, object] = dict(
        ut=0.0,
        met=100.0,
        position=Vector3(100.0, 200.0, 800.0),
        velocity=Vector3(10.0, -5.0, -30.0),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        altitude=800.0,
        surface_altitude=800.0,
        mass=30000.0,
        dry_mass=10000.0,
        thrust=0.0,
        available_thrust=400000.0,
        max_thrust=450000.0,
        max_vacuum_thrust=460000.0,
        specific_impulse=300.0,
        max_acceleration=0.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.0,
        frame=None,
    )
    data.update(overrides)
    return FlightState(**data)  # type: ignore[arg-type]


def _traj(u0: list[float], time_points: np.ndarray) -> SimpleNamespace:
    return SimpleNamespace(u_values=np.asarray([u0], dtype=float), time_points=time_points)


def test_build_config_auto_derivation() -> None:
    cfg = build_config(_state(), G_SURF, GfoldParams())
    data = json.loads(cfg.to_json())

    sc = data["spacecraft"]
    assert sc["wet_mass"] == 30000.0
    assert sc["fuel"] == 20000.0
    assert sc["real_max_thrust"] == 400000.0
    assert sc["min_thrust_pct"] == 0.2
    assert sc["max_thrust_pct"] == 1.0
    assert sc["initial_position"] == [100.0, 200.0, 800.0]
    assert sc["initial_velocity"] == [10.0, -5.0, -30.0]
    assert sc["fuel_consumption"] == pytest.approx(1.0 / (300.0 * G0))

    env = data["environment"]
    assert env["gravity"] == pytest.approx([0.0, 0.0, -G_SURF])
    assert env["glide_slope_angle_deg"] == 30.0
    assert env["max_angle_deg"] == 25.0

    sol = data["solver"]
    assert sol["n"] == 50
    assert sol["time_of_flight"] is None


def test_build_config_custom_target_and_tof() -> None:
    params = GfoldParams(
        target_position=(5.0, 6.0, 2.0),
        target_velocity=(0.0, 0.0, -1.0),
        max_angle_deg=12.0,
        glide_slope_angle_deg=45.0,
        min_throttle=0.3,
        max_throttle=0.9,
        max_velocity=500.0,
        n=80,
        tof=20.0,
    )
    cfg = build_config(_state(), G_SURF, params)
    data = json.loads(cfg.to_json())
    assert data["spacecraft"]["target_position"] == [5.0, 6.0, 2.0]
    assert data["spacecraft"]["target_velocity"] == [0.0, 0.0, -1.0]
    assert data["spacecraft"]["min_thrust_pct"] == 0.3
    assert data["spacecraft"]["max_thrust_pct"] == 0.9
    assert data["spacecraft"]["max_velocity"] == 500.0
    assert data["environment"]["max_angle_deg"] == 12.0
    assert data["environment"]["glide_slope_angle_deg"] == 45.0
    assert data["solver"]["n"] == 80
    assert data["solver"]["time_of_flight"] == 20.0


def test_build_config_tof_override() -> None:
    cfg = build_config(_state(), G_SURF, GfoldParams(tof=20.0), tof=5.0, n=40)
    data = json.loads(cfg.to_json())
    assert data["solver"]["time_of_flight"] == 5.0
    assert data["solver"]["n"] == 40


def test_command_throttle_and_direction() -> None:
    traj = _traj([3.0, 4.0, 0.0], np.linspace(0.0, 1.0, 10))
    throttle, direction = command(
        traj, mass=2000.0, available_thrust=20000.0, min_throttle=0.2
    )
    assert throttle == pytest.approx(0.5)  # |u|=5 -> 5*2000/20000
    # thrust acceleration coincides with the nose direction
    assert direction == pytest.approx((0.6, 0.8, 0.0))


def test_command_zero_thrust_defaults_up() -> None:
    traj = _traj([0.0, 0.0, 0.0], np.linspace(0.0, 1.0, 10))
    throttle, direction = command(
        traj, mass=2000.0, available_thrust=20000.0, min_throttle=0.2
    )
    assert throttle == 0.0
    assert direction == (0.0, 0.0, 1.0)


def test_command_clamps_throttle() -> None:
    traj = _traj([100.0, 0.0, 0.0], np.linspace(0.0, 1.0, 10))
    throttle, _ = command(
        traj, mass=2000.0, available_thrust=10000.0, min_throttle=0.2, max_throttle=1.0
    )
    assert throttle == 1.0  # |u|=100 -> 100*2000/10000 = 20 -> clamped


def test_command_clamps_nose_angle() -> None:
    traj = _traj([1.0, 0.0, 0.0], np.linspace(0.0, 1.0, 10))
    _, direction = command(
        traj,
        mass=2000.0,
        available_thrust=20000.0,
        max_angle_deg=30.0,
    )
    # horizontal thrust exceeds the 30 deg cone -> pulled to the cone edge
    assert direction == pytest.approx((np.sin(np.radians(30.0)), 0.0, np.cos(np.radians(30.0))))

    _, direction = command(
        traj,
        mass=2000.0,
        available_thrust=20000.0,
        max_angle_deg=180.0,
    )
    assert direction == pytest.approx((1.0, 0.0, 0.0))


def test_command_at_time_interpolates_and_clamps() -> None:
    u = np.asarray(
        [[0.0, 0.0, 5.0], [4.0, 0.0, 0.0], [0.0, 0.0, 8.0]], dtype=float
    )
    tp = np.asarray([0.0, 1.0, 2.0], dtype=float)
    traj = SimpleNamespace(u_values=u, time_points=tp)

    # before t=0 -> first node
    throttle, direction = command_at_time(traj, -1.0, mass=1000.0, available_thrust=10000.0)
    assert throttle == pytest.approx(0.5)  # |u|=5 -> 5*1000/10000
    assert direction == pytest.approx((0.0, 0.0, 1.0))

    # midpoint -> linear interpolation of the acceleration vector u=[2,0,4]
    throttle, direction = command_at_time(traj, 1.5, mass=1000.0, available_thrust=10000.0)
    assert throttle == pytest.approx(np.sqrt(20.0) / 10.0)
    assert direction == pytest.approx((2.0, 0.0, 4.0) / np.sqrt(20.0))

    # past horizon -> last node
    throttle, direction = command_at_time(traj, 5.0, mass=1000.0, available_thrust=10000.0)
    assert throttle == pytest.approx(0.8)  # |u|=8 -> 8*1000/10000
    assert direction == pytest.approx((0.0, 0.0, 1.0))


def test_tof_of() -> None:
    n = 50
    tof = 30.0
    tp = np.linspace(0.0, tof * (n - 1) / n, n)
    assert tof_of(_traj([0.0, 0.0, 1.0], tp)) == pytest.approx(tof)


def test_solve_infeasible_returns_none(monkeypatch: pytest.MonkeyPatch) -> None:
    def _fail(_cfg: object) -> None:
        raise ValueError("solver status: PrimalInfeasible")

    monkeypatch.setattr("gfold.solve", _fail)
    assert solve(build_config(_state(), G_SURF, GfoldParams())) is None


def test_solve_optimal_forces_tof_none(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    def _fake(cfg: object) -> SimpleNamespace:
        captured["cfg"] = cfg
        return _traj([0.0, 0.0, 9.0], np.linspace(0.0, 1.0, 10))

    monkeypatch.setattr("gfold.solve", _fake)
    traj = solve_optimal(_state(), G_SURF, GfoldParams(tof=20.0))
    assert traj is not None
    data = json.loads(captured["cfg"].to_json())  # type: ignore[attr-defined]
    assert data["solver"]["time_of_flight"] is None


def test_replan_passes_shrinking_tof(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    def _fake(cfg: object) -> SimpleNamespace:
        captured["cfg"] = cfg
        return _traj([0.0, 0.0, 9.0], np.linspace(0.0, 1.0, 10))

    monkeypatch.setattr("gfold.solve", _fake)
    traj = replan(_state(), G_SURF, GfoldParams(), tof=3.2)
    assert traj is not None
    data = json.loads(captured["cfg"].to_json())  # type: ignore[attr-defined]
    assert data["solver"]["time_of_flight"] == 3.2


def test_solve_returns_none_on_runtime_error(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(cfg: object) -> None:
        raise RuntimeError("numerical failure")

    monkeypatch.setattr("gfold.solve", boom)
    assert solve(object()) is None
