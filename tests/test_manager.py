import time
from collections.abc import Callable
from typing import Any

import pytest

from recovery.ksp import ConnectionManager
from recovery.ksp.types import Situation, Vector3
from tests.fakes import FakeClient, FakeVessel


def _pump(client: FakeClient, predicate: Callable[[], Any], timeout: float = 3.0) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result is not None:
            return result
        time.sleep(0.005)
    return None


def test_snapshot_pipeline_and_controls(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = FakeClient([vessel], ut=1234.0)
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    with ConnectionManager(address="127.0.0.1") as km:
        handle = km.add_booster("b1", "Booster 1", telemetry_hz=50.0)
        km.start()
        state = _pump(client, lambda: km.snapshot("b1"))
        assert state is not None
        assert state.ut == 1234.0
        assert state.met == 100.0
        assert state.position == Vector3(10.0, 20.0, 30.0)
        assert state.situation == Situation.FLYING
        assert state.landed is False
        assert state.specific_impulse == pytest.approx(300.0)
        assert state.max_acceleration == pytest.approx(400000.0 / 30000.0)
        handle.controls.apply(throttle=0.5)
        assert vessel.control.throttle == 0.5
    assert client.closed is True


def test_abort_all(monkeypatch) -> None:
    vessel_a = FakeVessel(name="Booster 1")
    vessel_b = FakeVessel(name="Booster 2")
    client = FakeClient([vessel_a, vessel_b])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")
    km.add_booster("b2", "Booster 2")
    km.start()
    vessel_a.control.throttle = 0.8
    vessel_a.auto_pilot.engaged = True
    vessel_b.control.throttle = 0.9
    vessel_b.auto_pilot.engaged = True
    km.abort_all()
    assert vessel_a.control.throttle == 0.0
    assert vessel_a.auto_pilot.engaged is False
    assert vessel_b.control.throttle == 0.0
    assert vessel_b.auto_pilot.engaged is False
    km.close()


def test_unknown_vessel_raises_and_closes(monkeypatch) -> None:
    client = FakeClient([FakeVessel(name="Booster 1")])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    with pytest.raises(ValueError):
        km.add_booster("b1", "Does Not Exist")
    assert client.closed is True
    km.close()


def test_duplicate_booster_id(monkeypatch) -> None:
    client = FakeClient([FakeVessel(name="A"), FakeVessel(name="B")])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    km.add_booster("b1", "A")
    with pytest.raises(ValueError):
        km.add_booster("b1", "B")
    km.close()


def test_unknown_booster_id(monkeypatch) -> None:
    client = FakeClient([FakeVessel(name="A")])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    km.add_booster("b1", "A")
    with pytest.raises(KeyError):
        km.snapshot("nope")
    km.close()


def test_target_frame_registration(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = FakeClient([vessel])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    with ConnectionManager() as km:
        km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.4730633292066, lat=-0.185355657540052)
        km.start()
        assert km.frame("b1", "target") is not None
        assert km.frame("b1", "surface") is not None
        with pytest.raises(KeyError):
            km.frame("b1", "bogus")


def test_abort_all_after_close_is_noop(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = FakeClient([vessel])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")
    km.start()
    km.close()
    km.abort_all()


def test_close_cuts_thrust(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = FakeClient([vessel])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    handle = km.add_booster("b1", "Booster 1")
    km.start()
    handle.controls.apply(throttle=0.85)
    handle.controls.engage_auto_pilot()
    assert vessel.control.throttle == 0.85
    assert vessel.auto_pilot.engaged is True
    km.close()
    assert vessel.control.throttle == 0.0
    assert vessel.auto_pilot.engaged is False


def test_target_registration_after_start_rejected(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = FakeClient([vessel])
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")
    km.start()
    with pytest.raises(RuntimeError):
        km.register_target("b1", lon=0.0, lat=0.0)
    km.close()
