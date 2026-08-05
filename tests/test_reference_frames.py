from math import cos, radians, sin

from recovery.ksp.reference_frames import create_target_reference_frame
from tests.fakes import FakeBody, FakeReferenceFrame, FakeSpaceCenter


def test_reference_frame_below_sea_level() -> None:
    FakeReferenceFrame.calls = []
    body = FakeBody(equatorial_radius=600000.0, bedrock=-100.0, surface=50.0)
    space_center = FakeSpaceCenter([])
    lon, lat = -74.4730633292066, -0.185355657540052
    frame = create_target_reference_frame(space_center, body, lon, lat)
    calls = FakeReferenceFrame.calls
    assert len(calls) == 5
    assert calls[0][1] == (0.0, sin(-radians(lon / 2)), 0.0, cos(-radians(lon / 2)))
    assert calls[1][1] == (0.0, 0.0, sin(radians(lat / 2)), cos(radians(lat / 2)))
    assert calls[2][0] == (600000.0, 0.0, 0.0)
    assert calls[2][1] == (0.0, 0.0, 0.0, 1.0)
    assert calls[3][1] == (0.0, sin(radians(45)), 0.0, cos(radians(45)))
    assert calls[4][1] == (0.0, 0.0, sin(radians(45)), cos(radians(45)))
    assert frame == "frame-5"


def test_reference_frame_above_sea_level() -> None:
    FakeReferenceFrame.calls = []
    body = FakeBody(equatorial_radius=600000.0, bedrock=10.0, surface=50.0)
    space_center = FakeSpaceCenter([])
    frame = create_target_reference_frame(space_center, body, -74.47, -0.18)
    calls = FakeReferenceFrame.calls
    assert calls[2][0] == (600050.0, 0.0, 0.0)
    assert frame == "frame-5"
