import time

import pytest

from recovery.clock import FramePacer


def test_hz_validation() -> None:
    with pytest.raises(ValueError):
        FramePacer(hz=0)
    with pytest.raises(ValueError):
        FramePacer(hz=-1)


def test_tick_paces_to_frame_boundaries(monkeypatch: pytest.MonkeyPatch) -> None:
    times = iter([0.0, 0.10, 0.35, 0.60])
    monkeypatch.setattr(time, "monotonic", lambda: next(times))
    monkeypatch.setattr(time, "sleep", lambda seconds: None)
    pacer = FramePacer(hz=10.0)
    assert pacer.period == pytest.approx(0.1)
    assert pacer.tick() == pytest.approx(0.1)
    assert pacer.tick() == pytest.approx(0.2)
    assert pacer.tick() == pytest.approx(0.3)


def test_hz_changeable_at_runtime() -> None:
    pacer = FramePacer(hz=10.0)
    pacer.hz = 20.0
    assert pacer.period == pytest.approx(0.05)


def test_max_dt_caps_large_delay(monkeypatch: pytest.MonkeyPatch) -> None:
    """A stalled frame reports ``max_dt`` instead of the true elapsed time."""
    times = iter([0.0, 5.0, 7.0])  # origin, then a 5 s stall, then a 2 s stall
    monkeypatch.setattr(time, "monotonic", lambda: next(times))
    monkeypatch.setattr(time, "sleep", lambda seconds: None)
    pacer = FramePacer(hz=10.0, max_dt=0.5)
    assert pacer.tick() == pytest.approx(0.1)  # first frame = period
    assert pacer.tick() == pytest.approx(0.5)  # 5 s elapsed -> capped


def test_max_dt_validation() -> None:
    with pytest.raises(ValueError):
        FramePacer(hz=10.0, max_dt=0.0)
    with pytest.raises(ValueError):
        FramePacer(hz=10.0, max_dt=-1.0)
