"""Concurrency regression tests for the shared-state races found in the
thread-safety audit (stage 3).

These tests FAIL on the un-locked implementation (they widen the race
window with ``sleep`` inside the critical sections) and PASS once the
lifecycle locks are in place.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

import pytest

from recovery.ksp import ConnectionManager
from tests.fakes import FakeClient, FakeVessel


def _connect(monkeypatch: pytest.MonkeyPatch, *vessels: FakeVessel) -> FakeClient:
    client = FakeClient(list(vessels))
    monkeypatch.setattr("recovery.ksp.connection.krpc.connect", lambda **kw: client)
    monkeypatch.setattr("recovery.ksp.debug.krpc.connect", lambda **kw: client)
    return client


def _run_threads(n: int, fn: Callable[[], Any]) -> list[threading.Thread]:
    """Start *n* threads that all hit *fn* simultaneously at a barrier."""
    barrier = threading.Barrier(n)

    def wrapped() -> None:
        barrier.wait()
        fn()

    threads = [threading.Thread(target=wrapped) for _ in range(n)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return threads


def test_body_spec_lazy_init_is_singleton(monkeypatch: pytest.MonkeyPatch) -> None:
    """Concurrent first access to ``body_spec`` must produce exactly one spec.

    The un-locked double-checked lazy init lets two threads both observe
    ``_body_spec is None`` and sample twice (two RPC passes, two instances).
    """
    import recovery.ksp.sampling as sampling
    import recovery.ksp.vessel as vessel_mod

    vessel = FakeVessel(name="Booster 1")
    _connect(monkeypatch, vessel)

    real_sample = sampling.sample_body_spec
    calls: list[int] = []

    def slow_sample(*args: Any, **kwargs: Any) -> Any:
        calls.append(1)
        time.sleep(0.02)  # widen the race window
        return real_sample(*args, **kwargs)

    # vessel.py binds sample_body_spec at import time, so patch it there.
    monkeypatch.setattr(vessel_mod, "sample_body_spec", slow_sample)

    km = ConnectionManager()
    handle = km.add_booster("b1", "Booster 1")
    km.register_target("b1", lon=-74.473, lat=-0.185)

    specs: list[Any] = []
    _run_threads(4, lambda: specs.append(handle.body_spec))

    # The expensive kRPC sampling must happen exactly once, no matter how many
    # threads race the lazy init.
    assert len(calls) == 1, f"body_spec sampled {len(calls)} times (want 1)"
    assert len({id(s) for s in specs}) == 1
    km.close()


def test_debug_proxy_lazy_init_is_singleton(monkeypatch: pytest.MonkeyPatch) -> None:
    """Concurrent first access to ``debug`` must build exactly one DebugProxy."""
    from recovery.ksp.debug import DebugProxy

    vessel = FakeVessel(name="Booster 1")
    _connect(monkeypatch, vessel)

    real_init = DebugProxy.__init__

    def slow_init(self: Any, *args: Any, **kwargs: Any) -> None:
        time.sleep(0.02)  # widen the race window
        real_init(self, *args, **kwargs)

    monkeypatch.setattr(DebugProxy, "__init__", slow_init)

    km = ConnectionManager()
    handle = km.add_booster("b1", "Booster 1")
    km.register_target("b1", lon=-74.473, lat=-0.185)
    km.enable_debug()

    proxies: list[Any] = []
    _run_threads(4, lambda: proxies.append(handle.debug))

    assert len({id(p) for p in proxies}) == 1, "debug proxy built more than once"
    km.close()


def test_start_concurrent_registers_streams_once(monkeypatch: pytest.MonkeyPatch) -> None:
    """Concurrent ``start()`` calls must launch the telemetry thread once.

    Each ``Telemetry._register_streams`` call appends 28 streams to the
    client; an un-locked ``_started`` check lets two threads both pass and
    register a second telemetry (streams count doubles).
    """
    vessel = FakeVessel(name="Booster 1")
    client = _connect(monkeypatch, vessel)

    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")

    _run_threads(4, km.start)

    assert len(client.streams) == 28, (
        f"telemetry streams={len(client.streams)}; a duplicate start doubles them"
    )
    km.close()


def test_snapshot_while_close_does_not_raise(monkeypatch: pytest.MonkeyPatch) -> None:
    """Readers racing ``close()`` must never observe a torn connection state.

    Regression guard: the snapshot path reads ``_telemetry`` while ``close``
    may null it; the telemetry ``stop()`` must also not crash when racing the
    publisher thread.
    """
    vessel = FakeVessel(name="Booster 1")
    _connect(monkeypatch, vessel)

    km = ConnectionManager()
    km.add_booster("b1", "Booster 1")
    km.start()

    stop = threading.Event()
    errors: list[BaseException] = []

    def reader() -> None:
        while not stop.is_set():
            try:
                km.snapshot("b1")
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)

    thread = threading.Thread(target=reader)
    thread.start()
    time.sleep(0.01)
    km.close()
    stop.set()
    thread.join()

    assert errors == []


def test_drawable_registration_atomic_with_clear_all(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A drawable created while ``clear_all()`` runs must never be orphaned.

    The un-locked implementation appends to ``_owned`` outside the RLock:
    if ``clear_all()`` replaces ``_owned = []`` while a draw is in flight, the
    new drawable lands on the discarded list and can never be cleared.
    """
    vessel = FakeVessel(name="Booster 1")
    client = _connect(monkeypatch, vessel)

    km = ConnectionManager()
    handle = km.add_booster("b1", "Booster 1")
    km.register_target("b1", lon=-74.473, lat=-0.185)
    km.enable_debug()
    proxy = handle.debug

    # Widen the window: hold the draw inside add_line so clear_all() can run
    # and replace _owned while the drawable is still being created.
    drawing = client.drawing
    real_add_line = drawing.add_line

    def slow_add_line(*args: Any, **kwargs: Any) -> Any:
        time.sleep(0.02)
        return real_add_line(*args, **kwargs)

    monkeypatch.setattr(drawing, "add_line", slow_add_line)

    barrier = threading.Barrier(2)
    created: list[Any] = []

    def drawer() -> None:
        barrier.wait()
        created.append(proxy.reference_frame(length=5.0))

    def clearer() -> None:
        barrier.wait()
        proxy.clear_all()

    t1 = threading.Thread(target=drawer)
    t2 = threading.Thread(target=clearer)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

    # Whatever the interleaving, the marker must be registered (either cleared
    # by the racing clear_all or still tracked) — a final clear_all() must
    # therefore remove every line from the scene.
    proxy.clear_all()
    assert all(line.removed for line in client.drawing.lines), (
        "a drawable created during clear_all() was orphaned"
    )
    km.close()
