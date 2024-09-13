from typing import Tuple, Set, Dict, List, TYPE_CHECKING, Optional, Any, Callable
from krpc.spacecenter import SpaceCenter


class DockingCamera:

    @property
    def available(self):
        # type: () -> bool
        """Check if the Camera API is avaiable"""
        ...

    def camera(self, part):
        # type: (SpaceCenter.Part) -> DockingCamera.Camera
        """Get a Camera part"""
        ...

    class Camera:
        @property
        def image(self):
            # type: () -> bytes
            """Get an image.
            Returns an empty byte array on failure."""
            ...

        @property
        def part(self):
            # type: () -> SpaceCenter.Part
            """Get the part containing this camera."""
            ...
