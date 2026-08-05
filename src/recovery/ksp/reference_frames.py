"""Reference frame construction for the KSP isolation layer."""

from __future__ import annotations

from math import cos, radians, sin
from typing import Any


def create_target_reference_frame(
    space_center: Any,
    body: Any,
    target_lon: float,
    target_lat: float,
) -> Any:
    """Create a landing-site reference frame for a target longitude/latitude.

    The axis convention is inherited verbatim from the legacy implementation
    (origin at the surface point of the target, orientation built from a
    longitude/latitude rotation of the body frame followed by two 90 degree
    spins). Do not reinterpret these axes; the guidance solver owns their
    meaning.
    """
    body_reference_frame = body.reference_frame
    temp_reference_frame = space_center.ReferenceFrame.create_relative(
        body_reference_frame,
        rotation=(0.0, sin(-radians(target_lon / 2)), 0.0, cos(-radians(target_lon / 2))),
    )
    temp_reference_frame = space_center.ReferenceFrame.create_relative(
        temp_reference_frame,
        rotation=(0.0, 0.0, sin(radians(target_lat / 2)), cos(radians(target_lat / 2))),
    )
    if body.bedrock_height(target_lat, target_lon) < 0:
        target_reference_frame_height = body.equatorial_radius
    else:
        target_reference_frame_height = body.equatorial_radius + body.surface_height(
            target_lat, target_lon
        )
    reference_frame = space_center.ReferenceFrame.create_relative(
        temp_reference_frame,
        position=(target_reference_frame_height, 0.0, 0.0),
    )
    temp_reference_frame = space_center.ReferenceFrame.create_relative(
        reference_frame,
        rotation=(0.0, sin(radians(45)), 0.0, cos(radians(45))),
    )
    reference_frame = space_center.ReferenceFrame.create_relative(
        temp_reference_frame,
        rotation=(0.0, 0.0, sin(radians(45)), cos(radians(45))),
    )
    return reference_frame
