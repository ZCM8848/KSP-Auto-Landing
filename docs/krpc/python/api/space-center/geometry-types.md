# Geometry Types

## Vectors

3-dimensional vectors are represented as a 3-tuple. For example:

```py
import krpc

conn = krpc.connect()
v = conn.space_center.active_vessel.flight().prograde
print(v[0], v[1], v[2])
```

## Quaternions

Quaternions (rotations in 3-dimensional space) are encoded as a 4-tuple
containing the x, y, z and w components. For example:

```py
import krpc

conn = krpc.connect()
q = conn.space_center.active_vessel.flight().rotation
print(q[0], q[1], q[2], q[3])
```
