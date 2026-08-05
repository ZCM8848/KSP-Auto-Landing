"""Live integration tests: require a running KSP with the kRPC server.

Run with:  python -m pytest -m live
Vessel name defaults to the KSP_VESSEL environment variable or "Booster 1".
"""

from __future__ import annotations

import os
import time

import pytest

from recovery.ksp import ConnectionManager

pytestmark = pytest.mark.live


def _wait_for_snapshot(km: ConnectionManager, booster_id: str, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = km.snapshot(booster_id)
        if state is not None:
            return state
        time.sleep(0.02)
    raise AssertionError(f"no telemetry snapshot within {timeout}s")


def test_snapshot_control_and_abort() -> None:
    vessel_name = os.environ.get("KSP_VESSEL", "Booster 1")
    with ConnectionManager(address="127.0.0.1") as km:
        booster = km.add_booster("b1", vessel_name)
        km.start()
        state = _wait_for_snapshot(km, "b1")
        assert state.mass > 0.0
        assert state.frame is not None
        booster.controls.apply(throttle=0.0)
        booster.controls.engage_auto_pilot()
        km.abort_all()
        assert booster.controls.throttle == 0.0
        assert booster.controls.auto_pilot_engaged is False
