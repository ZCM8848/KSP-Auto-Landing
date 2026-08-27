"""Landing-site coordinates as frozen data classes."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LandingSite:
    """Longitude and latitude in degrees."""

    lon: float
    lat: float


# ── Stock Kerbin ──────────────────────────────────────────

LAUNCHPAD_STOCK = LandingSite(-74.557673364044, -0.0972078545440865)
LZ1_STOCK = LandingSite(-74.4730633292066, -0.185355657540052)
LZ2_STOCK = LandingSite(-74.4729967713462, -0.20551670559373)
LZ3_STOCK = LandingSite(-74.4853049576317, -0.195548763275873)
DESSERT_STOCK = LandingSite(-143.950028091137, -6.56037689931713)

# ── JNSQ Kerbin ───────────────────────────────────────────

LAUNCHPAD_JNSQ = LandingSite(-91.7839786112259, 5.1753303155099e-06)
VAB_A_JNSQ = LandingSite(-91.8063860071064, -4.23555000546582e-06)
LZ1_JNSQ = LandingSite(-91.7519823454452, -0.0328509273631977)
LZ2_JNSQ = LandingSite(-91.7519474597292, -0.0404366898365102)
LZ3_JNSQ = LandingSite(-91.7565550827875, -0.0366963863349017)
LANDSPACE_LZ = LandingSite(103.92556780596897, 9.709217802785524)
