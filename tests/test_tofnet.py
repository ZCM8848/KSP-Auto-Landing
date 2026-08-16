"""Tests for the pure TOF-Net inference module."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from recovery.guidance.gfold import GfoldParams, features_of
from recovery.guidance.tofnet import FEATURES, TofPredictor
from recovery.types import FlightState, Quaternion, Situation, Vector3


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


def test_features_of_order_and_values() -> None:
    """features_of emits the 14-dim vector in the training-contract order."""
    state = _state(
        dry_mass=10000.0,
        mass=30000.0,
        available_thrust=400000.0,
        specific_impulse=300.0,
    )
    params = GfoldParams(
        min_throttle=0.3,
        max_throttle=0.9,
        glide_slope_angle_deg=45.0,
        max_angle_deg=12.0,
    )
    f = features_of(state, params)
    assert len(f) == 14
    assert f == pytest.approx(
        [
            100.0, 200.0, 800.0,  # position x, y, z
            10.0, -5.0, -30.0,  # velocity vx, vy, vz
            10000.0,  # dry_mass
            20000.0,  # fuel = mass - dry_mass
            400000.0,  # real_max_thrust
            0.3, 0.9,  # min/max throttle fraction
            1.0 / (300.0 * 9.80665),  # fuel_consumption = 1 / (Isp * G0)
            45.0, 12.0,  # glide slope, max pointing angle
        ]
    )


def test_features_of_thrust_fallback() -> None:
    """available_thrust == 0 falls back to max_thrust."""
    state = _state(available_thrust=0.0, max_thrust=450000.0)
    f = features_of(state, GfoldParams())
    assert f[8] == 450000.0


def _fake_session() -> MagicMock:
    sess = MagicMock()
    sess.get_inputs.return_value = [SimpleNamespace(name="features")]
    sess.get_outputs.return_value = [
        SimpleNamespace(name="p_feasible"),
        SimpleNamespace(name="tf"),
    ]
    sess.run.return_value = [
        np.array([[0.9]], dtype=np.float32),
        np.array([[42.0]], dtype=np.float32),
    ]
    return sess


def test_predictor_load_and_predict(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """TofPredictor loads metadata and runs inference through the session."""
    sess = _fake_session()
    fake_ort = SimpleNamespace(InferenceSession=lambda *a, **k: sess)
    monkeypatch.setitem(sys.modules, "onnxruntime", fake_ort)

    meta = {"features": list(FEATURES), "threshold": 0.48}
    meta_path = tmp_path / "tofnet.onnx.json"
    meta_path.write_text(json.dumps(meta), encoding="utf-8")
    onnx_path = tmp_path / "tofnet.onnx"

    pred = TofPredictor(onnx_path=str(onnx_path), meta_path=str(meta_path))

    assert pred.threshold == pytest.approx(0.48)
    assert pred.features == FEATURES

    p, tf = pred.predict([0.0] * 14)
    assert p == pytest.approx(0.9)
    assert tf == pytest.approx(42.0)
    sess.run.assert_called_once()
