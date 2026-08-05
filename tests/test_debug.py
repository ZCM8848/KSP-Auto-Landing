import pytest

from recovery.ksp import ConnectionManager
from tests.fakes import FakeClient, FakeVessel


def _connect(monkeypatch, *vessels) -> FakeClient:
    client = FakeClient(list(vessels))
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    monkeypatch.setattr("recovery.ksp.debug.krpc.connect", lambda **kw: client)
    return client


def test_debug_requires_enable(monkeypatch) -> None:
    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")
    with pytest.raises(RuntimeError):
        _ = km.vessel("b1").debug
    km.close()
    assert client.closed is True


def test_reference_frame_axes(monkeypatch) -> None:
    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.473, lat=-0.185)
        km.enable_debug()
        marker = km.vessel("b1").debug.reference_frame(frame_name="target", length=10.0)
        assert len(client.drawing.lines) == 3
        colors = {line.color for line in client.drawing.lines}
        assert colors == {(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)}
        marker.clear()
        assert all(line.removed for line in client.drawing.lines)


def test_trajectory_create_update_clear(monkeypatch) -> None:
    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.473, lat=-0.185)
        km.enable_debug()
        proxy = booster.debug
        traj = proxy.trajectory([(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3)], name="predicted")
        assert traj is not None
        assert len(client.drawing.lines) == 3
        proxy.trajectory([(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)], name="predicted")
        assert len(client.drawing.lines) == 3
        assert proxy.trajectory("predicted") is traj
        assert proxy.trajectories == {"predicted": traj}
        proxy.trajectory([(0, 0, 0), (5, 5, 5)], name="predicted")
        live = [line for line in client.drawing.lines if not line.removed]
        assert len(live) == 1
        proxy.clear("predicted")
        assert [line for line in client.drawing.lines if not line.removed] == []
        assert proxy.trajectory("predicted") is None


def test_trajectory_numpy_input(monkeypatch) -> None:
    import numpy as np

    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.473, lat=-0.185)
        km.enable_debug()
        points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        booster.debug.trajectory(points, name="t")
        lines = [line for line in client.drawing.lines if not line.removed]
        assert len(lines) == 2
        assert lines[0].start == (0.0, 0.0, 0.0)


def test_direction_and_line(monkeypatch) -> None:
    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.473, lat=-0.185)
        km.enable_debug()
        direction = booster.debug.direction((0.0, -1.0, 0.0), length=30.0, color=(1.0, 0.5, 0.0))
        line = booster.debug.line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), color=(0.0, 0.0, 1.0))
        assert len(client.drawing.lines) == 2
        assert client.drawing.lines[0].color == (1.0, 0.5, 0.0)
        direction.remove()
        line.visible = False
        assert client.drawing.lines[1].visible is False
        assert client.drawing.lines[0].removed is True


def test_clear_all(monkeypatch) -> None:
    client = _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.register_target("b1", lon=-74.473, lat=-0.185)
        km.enable_debug()
        proxy = booster.debug
        proxy.reference_frame()
        proxy.trajectory([(0, 0, 0), (1, 1, 1), (2, 2, 2)], name="t")
        proxy.clear_all()
        assert all(line.removed for line in client.drawing.lines)
        assert proxy.trajectories == {}


def test_unknown_frame(monkeypatch) -> None:
    _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.enable_debug()
        with pytest.raises(KeyError):
            booster.debug.reference_frame(frame_name="bogus")


def test_trajectory_requires_name(monkeypatch) -> None:
    _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.enable_debug()
        with pytest.raises(ValueError):
            booster.debug.trajectory([(0, 0, 0), (1, 1, 1)])


def test_target_required_before_draw(monkeypatch) -> None:
    _connect(monkeypatch, FakeVessel(name="Booster 1"))
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.enable_debug()
        with pytest.raises(RuntimeError):
            booster.debug.reference_frame(frame_name="target")


def test_vessel_surface_orbital_frames(monkeypatch) -> None:
    vessel = FakeVessel(name="Booster 1")
    client = _connect(monkeypatch, vessel)
    with ConnectionManager() as km:
        booster = km.add_booster("b1", "Booster 1")
        km.enable_debug()
        proxy = booster.debug
        for name in ("vessel", "surface", "orbital"):
            marker = proxy.reference_frame(frame_name=name, length=10.0)
            marker.clear()
        assert all(line.removed for line in client.drawing.lines)
