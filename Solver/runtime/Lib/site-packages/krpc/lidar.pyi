from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.spacecenter import SpaceCenter


class LiDAR:

    @property
    def available(self):
        # type: () -> bool
        """Check if the LaserDist API is avaiable"""
        ...

    def laser(self, part):
        # type: (SpaceCenter.Part) -> LiDAR.Laser
        """Get a LaserDist part"""
        ...

    class Laser:
        @property
        def cloud(self):
            # type: () -> List[float]
            """Get the point cloud from the LiDAR.
            Returns an empty list on failure."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """Get the part containing this LiDAR."""
            ...
