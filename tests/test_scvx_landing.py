"""Regression tests for helpers in ``scripts/scvx_landing.py``."""

import importlib.util
from pathlib import Path

import pytest

from recovery.guidance import (
    ControlledPredictor,
    EngineModel,
    TofFeatureParams,
    features_of,
)
from recovery.specs import BodySpec
from recovery.types import FlightState, Quaternion, Situation, Vector3

BODY = BodySpec(
    mu=3.5316e12,
    omega=(0.0, 0.0, 0.0),
    body_center=(0.0, 0.0, -600000.0),
    body_radius=600000.0,
    surface_gravity=9.81,
)


def _load_script():
    path = Path(__file__).resolve().parents[1] / "scripts" / "scvx_landing.py"
    spec = importlib.util.spec_from_file_location("scvx_landing_script", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _state() -> FlightState:
    return FlightState(
        ut=100.0,
        met=100.0,
        position=Vector3(0.0, 0.0, 500.0),
        velocity=Vector3(0.0, 0.0, -20.0),
        velocity_surface=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        direction=Vector3(0.0, 0.0, 1.0),
        bottom_axis=Vector3(1.0, 0.0, 0.0),
        altitude=500.0,
        surface_altitude=500.0,
        mass=1000.0,
        dry_mass=500.0,
        thrust=0.0,
        available_thrust=20000.0,
        max_thrust=20000.0,
        max_vacuum_thrust=20000.0,
        specific_impulse=300.0,
        max_acceleration=20.0,
        throttle=0.0,
        situation=Situation.FLYING,
        loaded=True,
        packed=False,
        landed=False,
        atmosphere_density=0.0,
        atmosphere_depth=70000.0,
        frame=None,
    )


def test_forward_state_keeps_vector3_types() -> None:
    """Latency compensation must preserve the FlightState type contract.

    A plain tuple here previously broke TOF-Net, whose ``features_of`` reads
    ``position.x`` / ``velocity.x``.
    """
    module = _load_script()
    state = _state()
    predictor = ControlledPredictor.from_body_spec(BODY)
    engine = EngineModel.from_state(state)

    aged = module._forward_state(
        state,
        age=0.1,
        predictor=predictor,
        engine=engine,
        last_command=None,
    )
    assert isinstance(aged.position, Vector3)
    assert isinstance(aged.velocity, Vector3)
    assert aged.ut == pytest.approx(state.ut + 0.1)
    assert aged.position.z < state.position.z

    features = features_of(aged, TofFeatureParams())
    assert len(features) == 14
