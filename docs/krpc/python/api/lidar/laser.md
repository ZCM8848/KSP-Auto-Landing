# Laser

class Laser
:   A LaserDist laser.

    part
    :   Get the part containing this LiDAR.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   [`SpaceCenter.Part`](../space-center/parts.md#SpaceCenter.Part "SpaceCenter.Part")

    cloud
    :   Get the point cloud from the LiDAR.
        Returns an empty list on failure.

        Attribute:
        :   Read-only, cannot be set

        Return type:
        :   list(float)
