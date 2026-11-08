"""TOF-Net inference: predict landing feasibility and optimal time-of-flight.

Wraps the pre-trained TOF-Net ONNX model behind a minimal, KSP-free API.  The
model is a black box — 14 raw physical features in, ``(p_feasible, tf)`` out.
All normalisation (min-max scaling, sigmoid, time-of-flight de-standardisation)
is baked into the exported ONNX graph, so this module only feeds raw features
through ``onnxruntime``.

The 14-feature order is the contract with the offline training pipeline (the
``GFOLD-solver`` project); :func:`features_of` is the canonical producer of that
vector from a :class:`~recovery.types.FlightState` and
:class:`TofFeatureParams`.

``onnxruntime`` is imported lazily inside :meth:`TofPredictor.__init__`, so
importing this module (and hence the whole ``guidance`` package) never requires
it to be installed.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

import numpy as np

from ..types import FlightState

G0 = 9.80665
"""Standard gravity (m/s²), used to convert specific impulse to the
``fuel_consumption`` feature ``1 / (Isp * G0)``.  This must match the value
the network was trained with — it is *not* the local surface gravity."""

#: Feature order — the training contract (see GFOLD-solver ``common/config.py``
#: ``FEATURES``).  :func:`features_of` must emit values in exactly this order.
FEATURES: tuple[str, ...] = (
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "dry_mass",
    "fuel",
    "real_max_thrust",
    "min_thrust_pct",
    "max_thrust_pct",
    "fuel_consumption",
    "glide_slope_angle_deg",
    "max_angle_deg",
)

_DEFAULT_ONNX = "tofnet.onnx"
_DEFAULT_META = "tofnet.onnx.json"


@dataclass(frozen=True)
class TofFeatureParams:
    """The mission-parameter columns of the 14-dim TOF-Net feature vector.

    These are the quantities the network needs but that are not part of the
    instantaneous flight state: the throttle envelope and the two guidance
    angles.  Defaults match the training pipeline's landing profile.
    """

    min_throttle: float = 0.2
    """Minimum thrust fraction (``real_max_thrust * min_throttle``)."""

    max_throttle: float = 1.0
    """Maximum thrust fraction (``real_max_thrust * max_throttle``)."""

    glide_slope_angle_deg: float = 30.0
    """Glide-slope cone half-angle about +z (deg)."""

    max_angle_deg: float = 25.0
    """Maximum angle between the thrust vector and +z (deg)."""


def _require_positive_isp(isp: float) -> float:
    """Return *isp* as a float, raising if it is not a positive finite value.

    A non-positive (or non-finite, e.g. NaN) specific impulse means the
    active engines are not firing (or the telemetry stream is stale).
    ``1 / (Isp * G0)`` would then either divide by zero or silently fabricate
    a nonsensical mass-flow rate, so we fail loudly instead of guessing a
    fallback.
    """
    isp = float(isp)
    if not math.isfinite(isp) or isp <= 0.0:
        raise ValueError(f"specific_impulse must be a positive finite value, got {isp}")
    return isp


def features_of(state: FlightState, params: TofFeatureParams) -> list[float]:
    """Return the 14-dim TOF-Net feature vector for *state* and *params*.

    The order is :data:`FEATURES`: position (x, y, z), velocity (vx, vy, vz),
    dry mass, fuel, max thrust, min/max throttle fraction, fuel consumption,
    glide-slope angle, max thrust-pointing angle.

    ``fuel_consumption`` uses the standard-gravity constant ``G0`` (Isp unit
    conversion), *not* the surface gravity — this must match the value the
    network was trained with.
    """
    p = state.position
    v = state.velocity
    isp = _require_positive_isp(state.specific_impulse)
    thrust = state.available_thrust if state.available_thrust > 0.0 else state.max_thrust
    return [
        p.x,
        p.y,
        p.z,
        v.x,
        v.y,
        v.z,
        state.dry_mass,
        state.mass - state.dry_mass,
        thrust,
        params.min_throttle,
        params.max_throttle,
        1.0 / (float(isp) * G0),
        params.glide_slope_angle_deg,
        params.max_angle_deg,
    ]


class TofPredictor:
    """Loads the TOF-Net ONNX model once and predicts ``(p_feasible, tf)``.

    The model is loaded on construction and kept resident in memory — never
    reloaded per frame.  ``onnxruntime`` is imported lazily inside
    :meth:`__init__` so the optional dependency is only needed when this class
    is actually instantiated.
    """

    def __init__(
        self,
        *,
        onnx_path: str | Path | None = None,
        meta_path: str | Path | None = None,
    ) -> None:
        """Load the ONNX model and its metadata.

        Keyword Args:
            onnx_path: Filesystem path to ``tofnet.onnx``.  Defaults to the
                packaged asset.  The external-weights file
                (``tofnet.onnx.data``) must sit alongside it.
            meta_path: Filesystem path to ``tofnet.onnx.json`` (features,
                threshold).  Defaults to the packaged asset.
        """
        if onnx_path is None:
            onnx_path = str(
                resources.files("recovery.guidance") / "assets" / _DEFAULT_ONNX
            )
        if meta_path is None:
            meta_path = str(
                resources.files("recovery.guidance") / "assets" / _DEFAULT_META
            )

        import onnxruntime as ort  # lazy: optional dependency

        self._sess = ort.InferenceSession(
            str(onnx_path), providers=["CPUExecutionProvider"]
        )
        self._input_name = self._sess.get_inputs()[0].name
        self._output_names = [o.name for o in self._sess.get_outputs()]

        meta = json.loads(Path(meta_path).read_text(encoding="utf-8"))
        self.threshold = float(meta["threshold"])
        """Feasibility threshold — ``p_feasible >= threshold`` means "landing
        is feasible".  Loaded from the model metadata."""

        self.features: tuple[str, ...] = tuple(meta["features"])
        """Feature names in model order (informational)."""

    def predict(self, features: Sequence[float]) -> tuple[float, float]:
        """Return ``(p_feasible, tf)`` for a raw 14-dim feature vector.

        Args:
            features: 14 raw physical values in the :data:`FEATURES` order
                (position, velocity, dry mass, fuel, max thrust, throttle
                bounds, fuel consumption, glide-slope angle, max
                thrust-pointing angle).

        Returns:
            ``(p_feasible, tf)`` — the feasibility probability in ``[0, 1]``
            and the predicted optimal time-of-flight in seconds.
        """
        x = np.asarray(features, dtype=np.float32)[None, :]
        p, tf = self._sess.run(self._output_names, {self._input_name: x})
        return float(np.asarray(p).ravel()[0]), float(np.asarray(tf).ravel()[0])
