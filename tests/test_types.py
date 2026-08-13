from types import SimpleNamespace

from recovery.types import Situation


def test_situation_mapping() -> None:
    assert Situation.from_krpc(SimpleNamespace(name="flying")) == Situation.FLYING
    assert Situation.from_krpc(SimpleNamespace(name="pre_launch")) == Situation.PRE_LAUNCH
    assert Situation.from_krpc(SimpleNamespace(name="landed")) == Situation.LANDED
    assert Situation.from_krpc(None) == Situation.UNKNOWN
    assert Situation.from_krpc(SimpleNamespace(name="bogus")) == Situation.UNKNOWN
