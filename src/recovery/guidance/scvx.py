"""Successive-convexification powered-descent planner (OpenSCvx backend).

This module builds the *end-to-end* descent optimal-control problem — the
unpowered coast and the landing burn are one trajectory, not two hand-written
phases — and runs it as an asynchronous model-predictive controller:

* :func:`build_descent_problem` assembles the OCP from the sampled
  :class:`~recovery.specs.BodySpec` / :class:`~recovery.specs.DragSpec` and the
  current :class:`~recovery.types.FlightState`.  Every vehicle/body-dependent
  quantity (gravity, atmosphere, drag area, thrust, Isp, mass, bounds) is
  derived from those inputs; the only literals are documented policy knobs in
  :class:`ScvxConfig`.
* :class:`ScvxPlanner` owns the OpenSCvx ``Problem`` on a worker thread.  It
  runs one SCP iteration at a time (~tens of ms), publishes each iterate as an
  immutable :class:`~recovery.guidance.plan.Plan`, and re-seeds the horizon from
  a fresh measurement when :meth:`ScvxPlanner.refresh` is called.  The control
  loop never blocks on the solver and keeps executing the latest feasible plan.

``openscvx`` is imported lazily (it is an optional, heavy JAX dependency), so
importing this module (and the ``guidance`` package) never requires it.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, cast

import numpy as np

from ..specs import BodySpec, DragSpec
from ..types import FlightState, TorquePair
from .plan import Plan

#: Standard gravity (m/s²): the Isp → mass-flow conversion constant.  A
#: physical constant, not a vehicle property.
G0 = 9.80665

#: Numerical floor for box bounds (m or m/s).  Prevents a degenerate zero-width
#: box when the vehicle starts exactly on the target with zero velocity.
_BOUND_FLOOR = 1e-3


@dataclass(frozen=True)
class ScvxConfig:
    """Policy knobs for the descent OCP.

    None of these describe a particular rocket: vehicle scales (mass, thrust,
    Isp, drag area) and body constants come from the sampled specs.  These are
    guidance-policy and numerical-conditioning choices.
    """

    n: int = 21
    """Number of transcription nodes over the whole horizon (coast + burn).

    The dynamics between nodes are integrated exactly (Dopri8); this number
    controls the control resolution (FOH) and the SCP problem size.
    """

    glide_slope_deg: float = 45.0
    """Half-angle of the approach cone about the local vertical (deg)."""

    max_tilt_deg: float = 30.0
    """Maximum angle between the thrust vector and local vertical (deg).

    Only used when :attr:`pointing_constraint` is true.
    """

    pointing_constraint: bool = True
    """Whether to constrain the thrust direction to the vertical cone."""

    horizon_factor: float = 2.0
    """Upper time bound as a multiple of the ballistic impact time."""

    horizon_min: float = 1.0
    """Lower time bound / degenerate-horizon floor (s)."""

    thrust_scaling: float = 0.8
    """SCP scaling bound for each thrust component, as a fraction of T_max."""

    bounds_margin: float = 0.1
    """Relative margin on position/velocity box bounds (unitless)."""

    velocity_margin: float = 0.2
    """Extra relative margin on the velocity box (unitless)."""

    float_dtype: str = "float64"
    """JAX precision; ``float64`` is measurably more robust for this OCP."""

    ode_solver: str = "Dopri8"
    """OpenSCvx discretizer ODE solver."""

    lam_cost: float = 1.0 / 3.0
    """SCP cost weight (matches the OpenSCvx PDG examples)."""

    lam_vc: float = 2.0
    """Virtual-control penalty weight."""

    lam_prox: float = 1.0
    """Proximal (trust-region) weight."""

    ep_tr: float | None = None
    """Optional trust-region convergence threshold (OpenSCvx ``ep_tr``)."""

    ep_vc: float | None = None
    """Optional virtual-control convergence threshold (OpenSCvx ``ep_vc``)."""

    max_iters_per_refresh: int = 5
    """SCP iterations to spend before waiting for the next measurement refresh.

    The worker publishes every iterate, so this bounds the latency of a
    refresh; warm-started horizons typically reach a usable plan in 1-3
    iterations and convergence in roughly twice that.
    """

    thrust_norm_epsilon: float = 1e-6
    """Regularisation of ``|T|`` as a fraction of ``T_max``.

    The mass-flow rate uses a smoothed norm so its derivative stays finite at
    zero thrust (the unpowered coast).  ``1e-6`` changes the thrust magnitude
    by far less than solver tolerance.
    """

    speed_norm_epsilon: float = 1e-3
    """Regularisation of ``|v|`` (m/s) for the drag term at zero airspeed."""

    quat_norm_tol: float = 1e-3
    """Allowed deviation of ``|q|²`` from 1 for 6-DOF attitude states."""

    min_throttle: float = 0.0
    """Minimum throttle fraction for the 6-DOF powered phase.

    Real engines cannot hold an arbitrarily small throttle: below the minimum
    they either cut off or run rough.  ``0`` keeps the original continuous
    ``[0, T_max]`` model; a positive value bounds the 6-DOF scalar thrust
    control to ``[min_throttle * T_max, T_max]`` (the engine never shuts down
    inside that horizon).  The 3-DOF builder ignores this knob, because its
    point-mass plan is allowed to coast.
    """

    multishot_plan: bool = False
    """Publish the multishot propagation as the plan (default: nodal iterate).

    The multishot plan is dynamically consistent even on early SCP iterates and
    is what the live controller should track.  The nodal iterate is kept as the
    default for backwards-compatible plan shapes (``n`` nodes)."""

    free_ignition: bool = False
    """Let the 6-DOF OCP choose the engine ignition time.

    The scalar thrust control becomes a burn-throttle fraction in ``[0, 1]``.
    A smooth gate around a free ``ignition_time`` state multiplies the engine
    thrust: zero before ignition, then ``[min_throttle, 1] * T_max``.  This is
    a continuous relaxation of the engine-on/off discontinuity; the gate width
    is :attr:`ignition_gate_width`.
    """

    ignition_gate_width: float = 0.5
    """Sigmoid gate width (s) for :attr:`free_ignition`."""

    ignition_guess: float = 15.0
    """Initial guess for the free ignition time (s after the horizon start)."""

    max_ignition_time: float | None = None
    """Optional upper bound on the free ignition time (s).

    A fuel-optimal free-ignition plan can postpone the burn until the vehicle
    is deep in the atmosphere.  On a vehicle whose RCS cannot hold attitude
    against the aerodynamic weathervane torque, that is not flyable.  Bounding
    the ignition time keeps the engine available before the dynamic pressure
    grows; ``None`` leaves the horizon bound unchanged.
    """

    max_descent_speed: float | None = None
    """Optional cap on downward speed (m/s, positive magnitude).

    Useful with :attr:`free_ignition`: an unconstrained fuel-optimal plan may
    delay ignition until the vehicle is falling very fast, which is hard to
    track laterally.  ``None`` leaves the velocity box unchanged.
    """


def _smooth_norm(vector: Any, epsilon: float, ox: Any) -> Any:
    """Differentiable norm ``sqrt(|v|² + ε²)``.

    ``jnp.linalg.norm`` has an undefined derivative at zero, which poisons the
    SCP linearisation when a coasting/stationary vehicle sits at ``|T| = 0`` or
    ``|v| = 0``.  This surrogate is smooth everywhere and differs from the true
    norm by less than ``ε``.
    """
    return ox.Sqrt(ox.Sum(vector * vector) + epsilon * epsilon)


def _algorithm_config(config: ScvxConfig, ox: Any) -> dict[str, Any]:
    """Assemble the OpenSCvx algorithm dictionary from :class:`ScvxConfig`."""
    algorithm: dict[str, Any] = {
        "autotuner": ox.ConstantProximalWeight(),
        "lam_cost": config.lam_cost,
        "lam_vc": config.lam_vc,
        "lam_prox": config.lam_prox,
    }
    if config.ep_tr is not None:
        algorithm["ep_tr"] = config.ep_tr
    if config.ep_vc is not None:
        algorithm["ep_vc"] = config.ep_vc
    return algorithm


@dataclass(frozen=True)
class EngineModel:
    """Current engine-set capability, sampled from telemetry."""

    available_thrust: float
    """Maximum thrust of the active engine set (N)."""

    isp: float
    """Specific impulse of the active engine set (s)."""

    dry_mass: float
    """Vessel dry mass (kg)."""

    @classmethod
    def from_state(cls, state: FlightState) -> EngineModel:
        """Derive the engine model from a telemetry snapshot."""
        thrust = state.available_thrust if state.available_thrust > 0.0 else state.max_thrust
        return cls(
            available_thrust=float(thrust),
            isp=float(state.specific_impulse),
            dry_mass=float(state.dry_mass),
        )


@dataclass(frozen=True)
class AttitudeModel:
    """Rigid-body inertia and torque authority in the vessel body frame.

    The body frame is the KSP vessel frame: ``+x`` right, ``+y`` nose,
    ``+z`` bottom.  Torque components are ordered the same way.
    """

    inertia: tuple[float, float, float]
    """Principal moments of inertia ``(Ixx, Iyy, Izz)`` (kg·m²)."""

    max_torque: tuple[float, float, float]
    """Maximum control torque per body axis (N·m)."""

    @classmethod
    def from_state(cls, state: FlightState) -> AttitudeModel:
        """Sum the available attitude-control torques from a snapshot.

        Reaction wheels, RCS, engine gimbal and control surfaces are treated as
        additive per axis; the sign-independent magnitude is used because the
        planner needs the authority envelope, not the current deflection.
        """
        moi = state.moment_of_inertia
        inertia = (
            max(float(moi.x), 1e-9),
            max(float(moi.y), 1e-9),
            max(float(moi.z), 1e-9),
        )

        def axis_authority(pair: TorquePair, index: int) -> float:
            negative = (pair.negative.x, pair.negative.y, pair.negative.z)[index]
            positive = (pair.positive.x, pair.positive.y, pair.positive.z)[index]
            return max(abs(float(negative)), abs(float(positive)))

        pairs = (
            state.available_reaction_wheel_torque,
            state.available_rcs_torque,
            state.available_engine_torque,
            state.available_control_surface_torque,
        )
        max_torque = (
            sum(axis_authority(pair, 0) for pair in pairs),
            sum(axis_authority(pair, 1) for pair in pairs),
            sum(axis_authority(pair, 2) for pair in pairs),
        )
        return cls(inertia=inertia, max_torque=max_torque)


def _quat_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Body→inertial rotation matrix from a unit quaternion ``(x, y, z, w)``."""
    q = np.asarray(quaternion, dtype=float)
    q = q / max(float(np.linalg.norm(q)), 1e-12)
    x, y, z, w = q
    return np.array(
        [
            [w * w + x * x - y * y - z * z, 2.0 * (x * y - w * z), 2.0 * (w * y + x * z)],
            [2.0 * (w * z + x * y), w * w - x * x + y * y - z * z, 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (w * x + y * z), w * w - x * x - y * y + z * z],
        ]
    )


def _quat_to_dcm_expr(quaternion: Any, ox: Any) -> Any:
    """Symbolic body→inertial rotation matrix from quaternion ``(x, y, z, w)``."""
    q1, q2, q3, q4 = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
    return ox.Block(
        [
            [
                q4 * q4 + q1 * q1 - q2 * q2 - q3 * q3,
                2.0 * (q1 * q2 - q4 * q3),
                2.0 * (q4 * q2 + q1 * q3),
            ],
            [
                2.0 * (q4 * q3 + q1 * q2),
                q4 * q4 - q1 * q1 + q2 * q2 - q3 * q3,
                2.0 * (q2 * q3 - q4 * q1),
            ],
            [
                2.0 * (q1 * q3 - q4 * q2),
                2.0 * (q4 * q1 + q2 * q3),
                q4 * q4 - q1 * q1 - q2 * q2 + q3 * q3,
            ],
        ]
    )


def _plane_gravity(body: BodySpec, position: np.ndarray) -> float:
    """Local gravitational acceleration magnitude at *position* (m/s²)."""
    radius = float(np.linalg.norm(np.asarray(position) - np.asarray(body.body_center)))
    return body.mu / max(radius * radius, 1.0)


def _ballistic_impact_time(state: FlightState, *, target_height: float, g_local: float) -> float:
    """Time to reach *target_height* under gravity alone (positive root)."""
    z = float(state.position.z) - float(target_height)
    if z <= 0.0:
        return 0.0
    vz = float(state.velocity.z)
    disc = vz * vz + 2.0 * g_local * z
    return float((vz + np.sqrt(max(disc, 0.0))) / max(g_local, _BOUND_FLOOR))


def _cross(a: Any, b: Any, ox: Any) -> Any:
    """Symbolic 3-vector cross product ``a × b``."""
    return ox.Concat(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


class DescentProblem:
    """A built OpenSCvx descent problem plus the helpers to drive it.

    Construct through :func:`build_descent_problem`.  All mutable state lives
    inside the wrapped ``openscvx.Problem``; this class only exposes the
    operations the planner needs.
    """

    def __init__(
        self,
        problem: Any,
        *,
        position: Any,
        velocity: Any,
        mass: Any,
        thrust: Any,
        n: int,
        config: ScvxConfig,
        g_local: float,
        target_height: float,
        time: Any,
        horizon_max: float,
        xy_limit: float,
        dof: str = "3dof",
        attitude: Any = None,
        angular_velocity: Any = None,
        torque: Any = None,
        attitude_final: np.ndarray | None = None,
        ignition_time: Any = None,
    ) -> None:
        self._problem = problem
        self._position = position
        self._velocity = velocity
        self._mass = mass
        self._thrust = thrust
        self._time = time
        self._n = int(n)
        self._config = config
        self._g_local = float(g_local)
        self._target_height = float(target_height)
        self._horizon_max = float(horizon_max)
        self._xy_limit = float(xy_limit)
        self._dof = dof
        self._attitude = attitude
        self._angular_velocity = angular_velocity
        self._torque = torque
        self._attitude_final = attitude_final
        self._ignition_time = ignition_time
        self._engine: EngineModel | None = None
        self._slices: dict[str, slice] | None = None
        self._last_plan_source: str = ""

    @property
    def dof(self) -> str:
        """Kinematic model used by this problem (``"3dof"`` or ``"6dof"``)."""
        return self._dof

    @property
    def free_ignition(self) -> bool:
        """Whether this problem optimizes a free engine ignition time."""
        return self._ignition_time is not None

    @property
    def xy_limit(self) -> float:
        """Half-width of the lateral position box used by the OCP (m).

        Derived from the current lateral offset/drift plus the lateral
        authority over the time-to-go; never collapses to a numerical floor
        just because the vehicle starts directly over the target.
        """
        return self._xy_limit

    @property
    def problem(self) -> Any:
        """The underlying ``openscvx.Problem`` (escape hatch)."""
        return self._problem

    @property
    def config(self) -> ScvxConfig:
        """The solver policy this problem was built with."""
        return self._config

    @property
    def slices(self) -> dict[str, slice]:
        """State/control slices in the flattened OpenSCvx vectors."""
        if self._slices is None:
            self._slices = dict(self._problem.slices)
        return self._slices

    @property
    def last_plan_source(self) -> str:
        """How the last :meth:`read_plan` was produced (diagnostics)."""
        return self._last_plan_source

    def initialize(self) -> None:
        """Compile the problem (JAX lowering; one-time cost of a few seconds)."""
        self._problem.initialize()

    def set_engine(self, engine: EngineModel) -> None:
        """Update the engine-set parameters (thrust limit, Isp)."""
        self._engine = engine
        self._problem.parameters["thrust_max"] = float(engine.available_thrust)
        self._problem.parameters["isp"] = float(engine.isp)

    def set_attitude_model(self, attitude: AttitudeModel) -> None:
        """Update the rigid-body inertia and torque envelope (6-DOF problems)."""
        if self._dof != "6dof":
            return
        self._problem.parameters["inertia"] = np.asarray(attitude.inertia, dtype=float)
        self._problem.parameters["torque_max"] = np.asarray(attitude.max_torque, dtype=float)

    def set_time_hint(self, tf: float | None) -> None:
        """Seed the free-final-time guess (s), e.g. from TOF-Net.

        The horizon duration remains a decision variable; this only sets the
        initial time grid, which speeds up convergence when the prediction is
        close.  Non-positive/non-finite hints are ignored.
        """
        if tf is None:
            return
        value = float(tf)
        if not np.isfinite(value) or value <= 0.0:
            return
        value = min(max(value, self._config.horizon_min), self._horizon_max)
        self._time.guess = np.linspace(0.0, value, self._n).reshape(-1, 1)

    def set_accel_bias(self, bias: np.ndarray) -> None:
        """Update the constant acceleration-bias parameter (m/s², target frame).

        The bias is the model-mismatch estimate produced by
        :class:`~recovery.guidance.correction.TerminalCorrector`; it is added
        to the planned acceleration so the reduced model matches the plant.
        """
        bias_arr = np.asarray(bias, dtype=float).reshape(3)
        self._problem.parameters["accel_bias"] = bias_arr

    def set_initial(
        self,
        state: FlightState,
        *,
        previous: Plan | None = None,
        tf_hint: float | None = None,
    ) -> None:
        """Re-seed the boundary conditions and the trajectory guess.

        With a *previous* plan, the guess is that plan resampled from the new
        measurement time (warm start).  Without one, a generic linear guess
        plus a hover-thrust profile is used.  ``tf_hint`` seeds the free final
        time guess when supplied (e.g. from TOF-Net).
        """
        self.set_time_hint(tf_hint)
        if self._ignition_time is not None:
            guess = float(self._config.ignition_guess)
            if tf_hint is not None and np.isfinite(tf_hint):
                guess = min(max(0.25 * float(tf_hint), 1.0), self._horizon_max)
            ignition_max = float(
                np.min(np.asarray(self._ignition_time.max, dtype=float))
            )
            guess = min(max(guess, 0.0), ignition_max)
            self._ignition_time.guess = np.full((self._n, 1), guess)
        p0 = np.asarray(state.position, dtype=float)
        v0 = np.asarray(state.velocity, dtype=float)
        m0 = float(state.mass)

        self._position.initial = p0
        self._velocity.initial = v0
        self._mass.initial = np.array([m0])

        if previous is not None:
            elapsed = float(state.ut) - previous.t0_ut
            remaining = max(previous.duration - elapsed, self._config.horizon_min)
            times = np.linspace(elapsed, elapsed + remaining, self._n)
            sampled = [previous.state_at(float(t)) for t in times]
            pos = np.array([sample[0] for sample in sampled])
            vel = np.array([sample[1] for sample in sampled])
            mass = np.array([max(sample[2], self._dry_mass) for sample in sampled])
            thrust = np.array([previous.thrust_at(float(t)) for t in times])
            pos[0] = p0
            vel[0] = v0
            mass[0] = m0
            peak = float(np.max(np.linalg.norm(thrust, axis=1)))
            limit = float(self._engine.available_thrust) if self._engine else peak
            if limit > 0.0 and peak > limit:
                thrust *= limit / peak
        else:
            target = np.array([0.0, 0.0, self._target_height])
            pos = np.linspace(p0, target, self._n)
            vel = np.linspace(v0, np.zeros(3), self._n)
            mass = np.full(self._n, m0)
            hover = m0 * self._g_local
            limit = float(self._engine.available_thrust) if self._engine else hover
            magnitude = min(hover, limit) if limit > 0.0 else 0.0
            thrust = np.tile(np.array([0.0, 0.0, magnitude]), (self._n, 1))

        self._position.guess = pos
        self._velocity.guess = vel
        self._mass.guess = mass.reshape(-1, 1)
        if self._dof == "6dof":
            self._set_six_dof_guess(
                state,
                m0,
                previous=previous,
                times=times if previous is not None else None,
                thrust_magnitude=(
                    np.linalg.norm(thrust, axis=1) if previous is not None else None
                ),
            )
        else:
            self._thrust.guess = thrust

    def _set_six_dof_guess(
        self,
        state: FlightState,
        mass: float,
        *,
        previous: Plan | None = None,
        times: np.ndarray | None = None,
        thrust_magnitude: np.ndarray | None = None,
    ) -> None:
        """Seed attitude/angular-velocity/thrust/torque guesses for 6-DOF.

        With a warm-start plan the attitude, angular-rate, thrust and torque
        histories are resampled from it.  The previous plan may be a 3-DOF
        (point-mass) plan, in which case only its thrust magnitude is used and
        the attitude guess stays at the measured vertical pose.
        """
        if self._attitude is None or self._angular_velocity is None or self._torque is None:
            raise RuntimeError("6-DOF problem is missing its attitude state objects")
        quaternion = np.array(
            [state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w],
            dtype=float,
        )
        quaternion /= max(float(np.linalg.norm(quaternion)), 1e-12)
        if (
            self._attitude_final is not None
            and float(np.dot(quaternion, self._attitude_final)) < 0.0
        ):
            quaternion = -quaternion
        body_rate = _quat_to_matrix(quaternion).T @ np.asarray(
            [state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z],
            dtype=float,
        )

        self._attitude.initial = quaternion
        self._angular_velocity.initial = body_rate

        attitude_guess = np.tile(quaternion, (self._n, 1))
        angular_guess = np.tile(body_rate, (self._n, 1))
        torque_guess = np.zeros((self._n, 3))
        if previous is not None and times is not None and previous.attitudes is not None:
            sampled_attitude = np.empty((self._n, 4), dtype=float)
            for index, time_value in enumerate(times):
                sampled = previous.attitude_at(float(time_value))
                sampled_attitude[index] = quaternion if sampled is None else sampled
            norms = np.linalg.norm(sampled_attitude, axis=1, keepdims=True)
            attitude_guess = sampled_attitude / np.maximum(norms, 1e-12)
            # Keep the quaternion sign continuous with the initial attitude.
            signs = np.sign(attitude_guess @ quaternion)
            signs[signs == 0.0] = 1.0
            attitude_guess *= signs[:, None]
        if previous is not None and times is not None and previous.angular_velocities is not None:
            table = np.asarray(previous.angular_velocities, dtype=float)
            angular_guess = np.stack(
                [
                    np.interp(times, previous.times, table[:, axis])
                    for axis in range(3)
                ],
                axis=1,
            )
        if previous is not None and times is not None and previous.torques is not None:
            table = np.asarray(previous.torques, dtype=float)
            torque_guess = np.stack(
                [
                    np.interp(times, previous.times, table[:, axis])
                    for axis in range(3)
                ],
                axis=1,
            )

        self._attitude.guess = attitude_guess
        self._angular_velocity.guess = angular_guess

        hover = mass * self._g_local
        if self._engine is not None:
            t_max = float(self._engine.available_thrust)
            t_min = self._config.min_throttle * t_max
            hover = min(max(hover, t_min), t_max)
        if thrust_magnitude is None:
            thrust_guess = np.full(self._n, hover)
        else:
            thrust_guess = np.asarray(thrust_magnitude, dtype=float)
            if self._engine is not None:
                t_max = float(self._engine.available_thrust)
                t_min = self._config.min_throttle * t_max
                thrust_guess = np.clip(thrust_guess, t_min, t_max)
        if self._config.free_ignition:
            # The control is a burn-throttle fraction, not a force.
            self._thrust.guess = np.full((self._n, 1), 0.6)
        else:
            self._thrust.guess = thrust_guess.reshape(-1, 1)
        self._torque.guess = torque_guess

    @property
    def _dry_mass(self) -> float:
        if self._engine is None:
            raise RuntimeError("engine model not set; call set_engine() first")
        return float(self._engine.dry_mass)

    def reset(self) -> None:
        """Reset the SCP state, syncing guesses and boundary conditions."""
        self._problem.reset()

    def step(self) -> dict[str, Any]:
        """Run one SCP iteration and return OpenSCvx's convergence dict."""
        return cast("dict[str, Any]", self._problem.step())

    def read_plan(self, t0_ut: float, *, converged: bool = False) -> Plan:
        """Snapshot the current iterate as an immutable :class:`Plan`.

        The nodal optimization variables in ``state.x`` can still violate the
        nonlinear dynamics before SCP convergence, so the plan is built from
        the *multishot propagation* recorded in ``history.V_history``: those
        substates are the result of integrating the current controls through
        the nonlinear dynamics and are therefore dynamically consistent even
        on early iterations.  This is what keeps the real-time controller from
        tracking a mathematically convenient but physically impossible node
        sequence.
        """
        state = self._problem.state
        if state is None:
            raise RuntimeError("problem has no state; call initialize() first")
        if self._config.multishot_plan:
            plan = self._read_multishot_plan(t0_ut, state, converged)
            if plan is not None:
                self._last_plan_source = "multishot"
                return plan

        # Fallback for an OpenSCvx build without multishot history (or when
        # the legacy nodal plan shape is requested): use the raw nodal iterate.
        # The caller should treat this as a coarse plan unless SCP converged.
        self._last_plan_source = (
            "nodal-fallback" if self._config.multishot_plan else "nodal"
        )
        x = np.asarray(state.x, dtype=float)
        u = np.asarray(state.u, dtype=float)
        sl = self.slices
        times = np.array(x[:, sl["time"]], dtype=float).reshape(-1)
        times = times - times[0]
        cost = float(getattr(state, "J_tr", float("nan")))
        if self._dof == "6dof":
            return self._read_six_dof_plan(t0_ut, x, u, sl, times, cost, converged)
        return Plan(
            t0_ut=float(t0_ut),
            times=times,
            positions=np.array(x[:, sl["position"]], dtype=float),
            velocities=np.array(x[:, sl["velocity"]], dtype=float),
            masses=np.array(x[:, sl["mass"]], dtype=float).reshape(-1),
            thrusts=np.array(u[:, sl["thrust"]], dtype=float),
            iterations=int(getattr(state, "k", 0)),
            cost=cost,
            converged=bool(converged),
        )

    def _read_multishot_plan(
        self,
        t0_ut: float,
        state: Any,
        converged: bool,
    ) -> Plan | None:
        """Build a dynamically consistent plan from the multishot history."""
        try:
            from openscvx.algorithms.history import unpack_multishot_V
        except Exception:  # pragma: no cover - only on an incompatible build
            return None
        history = getattr(self._problem, "history", None)
        v_history = getattr(history, "V_history", None)
        if not v_history:
            return None
        V = np.asarray(v_history[-1], dtype=float)
        sim = self._problem.settings.sim
        n_x = int(sim.n_states)
        n_u = int(sim.n_controls)
        if V.size == 0 or n_x <= 0 or n_u <= 0:
            return None

        x_nodes = np.asarray(state.x, dtype=float)
        u_nodes = np.asarray(state.u, dtype=float)
        sl = self.slices
        t_nodes = np.asarray(x_nodes[:, sl["time"]], dtype=float).reshape(-1)
        try:
            propagation = unpack_multishot_V(
                V, n_x=n_x, n_u=n_u, t_nodes=t_nodes
            )
            states, times = propagation.chronological()
        except Exception:  # pragma: no cover - defensive fallback
            return None
        if states.shape[0] < 2:
            return None

        # FOH control interpolation on the same physical-time grid as the
        # propagated substates.  This matches the executor's interpolation.
        controls = np.empty((times.shape[0], n_u), dtype=float)
        for index in range(n_u):
            controls[:, index] = np.interp(
                times, t_nodes, u_nodes[:, index], left=u_nodes[0, index],
                right=u_nodes[-1, index],
            )

        times = np.asarray(times, dtype=float)
        times = times - times[0]
        positions = np.asarray(states[:, sl["position"]], dtype=float)
        velocities = np.asarray(states[:, sl["velocity"]], dtype=float)
        masses = np.asarray(states[:, sl["mass"]], dtype=float).reshape(-1)
        thrust_scalar = np.asarray(controls[:, sl["thrust"]], dtype=float)
        ignition_time_value: float | None = None
        if self.free_ignition:
            t_series = np.asarray(states[:, sl["time"]], dtype=float).reshape(-1)
            ignition_series = np.asarray(
                states[:, sl["ignition_time"]], dtype=float
            ).reshape(-1)
            burn_throttle = np.asarray(
                controls[:, sl["thrust"]], dtype=float
            ).reshape(-1)
            if self._engine is not None:
                thrust_cap = float(self._engine.available_thrust)
            else:
                thrust_cap = 1.0
            thrust_min = self._config.min_throttle * thrust_cap
            width = max(float(self._config.ignition_gate_width), 1e-3)
            gate = 1.0 / (
                1.0 + np.exp(-(t_series - ignition_series) / width)
            )
            thrust_scalar = (
                gate * (thrust_min + (thrust_cap - thrust_min) * burn_throttle)
            ).reshape(-1, 1)
            ignition_time_value = float(ignition_series[0]) if ignition_series.size else None
        cost = float(getattr(state, "J_tr", float("nan")))
        iterations = int(getattr(state, "k", 0))

        if self._dof != "6dof":
            return Plan(
                t0_ut=float(t0_ut),
                times=times,
                positions=positions,
                velocities=velocities,
                masses=masses,
                thrusts=thrust_scalar,
                iterations=iterations,
                cost=cost,
                converged=bool(converged),
                ignition_time=ignition_time_value,
            )

        attitude = np.asarray(states[:, sl["attitude"]], dtype=float)
        norms = np.linalg.norm(attitude, axis=1, keepdims=True)
        attitude = attitude / np.maximum(norms, 1e-12)
        angular_velocity = np.asarray(states[:, sl["angular_velocity"]], dtype=float)
        directions = np.empty((attitude.shape[0], 3), dtype=float)
        for index, quaternion in enumerate(attitude):
            directions[index] = _quat_to_matrix(quaternion) @ np.array([0.0, 1.0, 0.0])
        thrusts = thrust_scalar[:, 0:1] * directions
        torques = np.asarray(controls[:, sl["torque"]], dtype=float)
        return Plan(
            t0_ut=float(t0_ut),
            times=times,
            positions=positions,
            velocities=velocities,
            masses=masses,
            thrusts=thrusts,
            iterations=iterations,
            cost=cost,
            converged=bool(converged),
            attitudes=attitude,
            angular_velocities=angular_velocity,
            torques=torques,
            ignition_time=ignition_time_value,
        )

    def _read_six_dof_plan(
        self,
        t0_ut: float,
        x: np.ndarray,
        u: np.ndarray,
        sl: dict[str, slice],
        times: np.ndarray,
        cost: float,
        converged: bool,
    ) -> Plan:
        """Build a :class:`Plan` from a 6-DOF iterate."""
        attitude = np.array(x[:, sl["attitude"]], dtype=float)
        norms = np.linalg.norm(attitude, axis=1, keepdims=True)
        attitude = attitude / np.maximum(norms, 1e-12)
        angular_velocity = np.array(x[:, sl["angular_velocity"]], dtype=float)
        thrust = np.array(u[:, sl["thrust"]], dtype=float).reshape(-1)
        torque = np.array(u[:, sl["torque"]], dtype=float)
        ignition_time_value: float | None = None
        if self.free_ignition:
            t_series = np.array(x[:, sl["time"]], dtype=float).reshape(-1)
            ignition_series = np.array(
                x[:, sl["ignition_time"]], dtype=float
            ).reshape(-1)
            if self._engine is not None:
                thrust_cap = float(self._engine.available_thrust)
            else:
                thrust_cap = 1.0
            thrust_min = self._config.min_throttle * thrust_cap
            width = max(float(self._config.ignition_gate_width), 1e-3)
            gate = 1.0 / (
                1.0 + np.exp(-(t_series - ignition_series) / width)
            )
            thrust = gate * (thrust_min + (thrust_cap - thrust_min) * thrust)
            ignition_time_value = (
                float(ignition_series[0]) if ignition_series.size else None
            )

        # Body +y is the nose; the inertial thrust direction is R(q) @ e_y.
        directions = np.empty((attitude.shape[0], 3), dtype=float)
        for index, quaternion in enumerate(attitude):
            directions[index] = _quat_to_matrix(quaternion) @ np.array([0.0, 1.0, 0.0])
        thrusts = thrust[:, None] * directions

        return Plan(
            t0_ut=float(t0_ut),
            times=times,
            positions=np.array(x[:, sl["position"]], dtype=float),
            velocities=np.array(x[:, sl["velocity"]], dtype=float),
            masses=np.array(x[:, sl["mass"]], dtype=float).reshape(-1),
            thrusts=thrusts,
            iterations=int(getattr(self._problem.state, "k", 0)),
            cost=cost,
            converged=bool(converged),
            attitudes=attitude,
            angular_velocities=angular_velocity,
            torques=torque,
            ignition_time=ignition_time_value,
        )


def build_descent_problem(
    body: BodySpec,
    drag: DragSpec,
    *,
    state: FlightState,
    target_height: float,
    engine: EngineModel,
    config: ScvxConfig | None = None,
) -> DescentProblem:
    """Build the end-to-end descent OCP for *state*.

    Args:
        body: Planetary constants in the target frame.
        drag: Sampled atmosphere + drag area.  ``drag.cd_area`` must be known
            (it is filled in by :func:`recovery.ksp.sampling.sample_drag_spec`
            when a mass is supplied).
        state: Current snapshot; supplies the initial condition and the
            vehicle-dependent scales used for bounds.
        target_height: Height of the target CoM above the target-frame origin
            (m).  The target-frame origin is the landing-site surface.
        engine: Active engine-set capability.
        config: Policy knobs (see :class:`ScvxConfig`).

    Raises:
        ValueError: if the drag area is unknown or the engine model is invalid.
        ImportError: if ``openscvx`` is not installed.
    """
    import openscvx as ox  # lazy: heavy optional dependency

    if config is None:
        config = ScvxConfig()
    if drag.cd_area is None or not np.isfinite(drag.cd_area):
        raise ValueError(
            "DragSpec.cd_area is unknown; sample the drag spec with a known "
            "vehicle mass so Cd·A can be derived"
        )
    if engine.available_thrust <= 0.0:
        raise ValueError("engine.available_thrust must be positive")
    if engine.isp <= 0.0:
        raise ValueError("engine.isp must be positive")

    n = int(config.n)
    if n < 3:
        raise ValueError(f"config.n must be >= 3, got {n}")

    p0 = np.asarray(state.position, dtype=float)
    v0 = np.asarray(state.velocity, dtype=float)
    m0 = float(state.mass)
    g_local = _plane_gravity(body, p0)
    t_impact = _ballistic_impact_time(state, target_height=target_height, g_local=g_local)
    atmosphere_top = float(drag.density_alts[-1]) if len(drag.density_alts) else 0.0
    # A descent that starts at the top of the atmosphere can take longer than
    # the current ballistic estimate (drag), so give the free final time enough
    # room for any in-atmosphere descent.  This is a physical bound, not a
    # vehicle-specific constant.
    horizon_max = max(
        config.horizon_factor * t_impact,
        2.0 * config.horizon_min,
    )
    tf_guess = max(config.horizon_min, t_impact)

    # --- bounds, derived from the current state and physics -----------------
    vz_up = max(float(v0[2]), 0.0)
    z_apex = float(p0[2]) + vz_up * vz_up / (2.0 * max(g_local, _BOUND_FLOOR))
    z_max = max(float(p0[2]), z_apex, float(target_height), atmosphere_top) * (
        1.0 + config.bounds_margin
    )
    t_go = max(t_impact, config.horizon_min)
    lateral_authority = float(engine.available_thrust) / m0
    if config.pointing_constraint:
        lateral_authority *= float(np.sin(np.radians(config.max_tilt_deg)))
    xy_limit = max(
        (
            float(np.max(np.abs(p0[:2])))
            + float(np.max(np.abs(v0[:2]))) * t_go
            + 0.5 * lateral_authority * t_go * t_go
        )
        * (1.0 + config.bounds_margin),
        _BOUND_FLOOR,
    )
    v_limit = (float(np.max(np.abs(v0))) + g_local * horizon_max) * (
        1.0 + config.velocity_margin
    )
    t_max = float(engine.available_thrust)

    position = ox.State("position", shape=(3,))
    position.min = np.array([-xy_limit, -xy_limit, 0.0])
    position.max = np.array([xy_limit, xy_limit, z_max])
    position.initial = p0
    position.final = np.array([0.0, 0.0, float(target_height)])
    position.guess = np.linspace(p0, position.final, n)

    velocity = ox.State("velocity", shape=(3,))
    velocity.min = np.array([-v_limit, -v_limit, -v_limit])
    velocity.max = np.array([v_limit, v_limit, v_limit])
    velocity.initial = v0
    velocity.final = np.zeros(3)
    velocity.guess = np.linspace(v0, velocity.final, n)

    mass = ox.State("mass", shape=(1,))
    mass.min = np.array([float(engine.dry_mass)])
    mass.max = np.array([m0])
    mass.initial = np.array([m0])
    mass.final = [("maximize", float(engine.dry_mass))]
    mass.guess = np.full((n, 1), m0)

    thrust = ox.Control("thrust", shape=(3,))
    thrust.min = -t_max * np.ones(3)
    thrust.max = t_max * np.ones(3)
    thrust.scaling_min = -config.thrust_scaling * t_max * np.ones(3)
    thrust.scaling_max = config.thrust_scaling * t_max * np.ones(3)
    thrust.guess = np.tile(np.array([0.0, 0.0, min(m0 * g_local, t_max)]), (n, 1))

    # --- parameters (updated between solves without recompiling) ------------
    thrust_max = ox.Parameter("thrust_max", value=t_max)
    isp = ox.Parameter("isp", value=float(engine.isp))
    accel_bias = ox.Parameter("accel_bias", shape=(3,), value=np.zeros(3))

    # --- dynamics -----------------------------------------------------------
    center = np.asarray(body.body_center, dtype=float)
    omega = np.asarray(body.omega, dtype=float)
    d = position - center
    radius = ox.linalg.Norm(d)
    gravity = -body.mu * d / (radius**3)
    rotation = -2.0 * _cross(omega, velocity, ox) - _cross(
        omega, _cross(omega, d, ox), ox
    )

    altitude = radius - float(drag.sea_level_radius)
    table_top = float(drag.density_alts[-1])
    # Clamp the lookup to the sampled table: no negative densities below the
    # surface and no cubic extrapolation above the atmosphere.
    density = ox.Max(
        ox.Cinterp(ox.Min(ox.Max(altitude, 0.0), table_top), drag.density_alts, drag.density_vals),
        0.0,
    )
    speed = _smooth_norm(velocity, config.speed_norm_epsilon, ox)
    drag_accel = (
        -0.5 * density * float(drag.cd_area) * speed * velocity / mass[0]
    )

    dynamics = {
        "position": velocity,
        "velocity": thrust / mass[0] + gravity + rotation + drag_accel + accel_bias,
        "mass": -_smooth_norm(thrust, config.thrust_norm_epsilon * t_max, ox) / (isp * G0),
    }

    # --- constraints --------------------------------------------------------
    constraints: list[Any] = []
    for component in (position, velocity, mass):
        constraints.extend(
            [
                ox.ctcs(component <= component.max, idx=0),
                ox.ctcs(component.min <= component, idx=0),
            ]
        )
    # Thrust magnitude and pointing are written in squared, normalised form:
    # the plain norm has an undefined derivative at |T| = 0, which the
    # unpowered coast reaches.  `scaled` is dimensionless, so the constraints
    # stay well conditioned regardless of vehicle size.
    scaled_thrust = thrust / thrust_max
    thrust_sq = ox.Sum(scaled_thrust * scaled_thrust)
    constraints.append(ox.ctcs(thrust_sq <= 1.0, idx=1))
    if config.pointing_constraint:
        cos_tilt = float(np.cos(np.radians(config.max_tilt_deg)))
        constraints.append(ox.ctcs(0.0 <= thrust[2], idx=2))
        constraints.append(
            ox.ctcs(cos_tilt * cos_tilt * thrust_sq <= (thrust[2] / thrust_max) ** 2, idx=2)
        )
    tan_slope = float(np.tan(np.radians(config.glide_slope_deg)))
    scale = max(float(p0[2]), float(target_height), 1.0)
    lateral = (position[0] / scale) ** 2 + (position[1] / scale) ** 2
    height_above_site = (position[2] - float(target_height)) / scale
    constraints.append(
        ox.ctcs(lateral <= (tan_slope * height_above_site) ** 2, idx=3)
    )

    time_axis = ox.Time(
        initial=0.0,
        final=ox.Free(tf_guess),
        min=0.0,
        max=horizon_max,
        guess=np.linspace(0.0, tf_guess, n).reshape(-1, 1),
        time_dilation_min=0.3 * tf_guess,
        time_dilation_max=3.0 * tf_guess,
    )

    problem = ox.Problem(
        dynamics=dynamics,
        states=[position, velocity, mass],
        controls=[thrust],
        time=time_axis,
        constraints=constraints,
        N=n,
        algorithm=_algorithm_config(config, ox),
        discretizer={"ode_solver": config.ode_solver},
        float_dtype=config.float_dtype,
    )
    problem.settings.dev.printing = False

    return DescentProblem(
        problem,
        position=position,
        velocity=velocity,
        mass=mass,
        thrust=thrust,
        n=n,
        config=config,
        g_local=g_local,
        target_height=target_height,
        time=time_axis,
        horizon_max=horizon_max,
        xy_limit=xy_limit,
    )


def build_six_dof_problem(
    body: BodySpec,
    drag: DragSpec,
    *,
    state: FlightState,
    target_height: float,
    engine: EngineModel,
    attitude: AttitudeModel | None = None,
    config: ScvxConfig | None = None,
) -> DescentProblem:
    """Build a 6-DOF descent OCP (quaternion attitude + body torque).

    The body frame matches the KSP vessel frame: ``+x`` right, ``+y`` nose,
    ``+z`` bottom.  Thrust acts along body ``+y``; the control torque is a
    body-frame vector bounded by the summed reaction-wheel/RCS/gimbal/control-
    surface authority.  This makes the attitude slew (and therefore the thrust
    direction) dynamically feasible instead of instantaneous.

    Args:
        body: Planetary constants in the target frame.
        drag: Sampled atmosphere + drag area (``cd_area`` required).
        state: Current snapshot; supplies the initial condition and scales.
        target_height: Terminal CoM height above the target-frame origin (m).
        engine: Active engine-set capability.
        attitude: Rigid-body inertia and torque envelope; derived from *state*
            when omitted.
        config: Policy knobs.
    """
    import openscvx as ox  # lazy: heavy optional dependency

    if config is None:
        config = ScvxConfig()
    if attitude is None:
        attitude = AttitudeModel.from_state(state)
    if drag.cd_area is None or not np.isfinite(drag.cd_area):
        raise ValueError(
            "DragSpec.cd_area is unknown; sample the drag spec with a known "
            "vehicle mass so Cd·A can be derived"
        )
    if engine.available_thrust <= 0.0:
        raise ValueError("engine.available_thrust must be positive")
    if engine.isp <= 0.0:
        raise ValueError("engine.isp must be positive")

    n = int(config.n)
    if n < 3:
        raise ValueError(f"config.n must be >= 3, got {n}")
    if not 0.0 <= config.min_throttle < 1.0:
        raise ValueError(
            f"config.min_throttle must be in [0, 1), got {config.min_throttle}"
        )

    p0 = np.asarray(state.position, dtype=float)
    v0 = np.asarray(state.velocity, dtype=float)
    m0 = float(state.mass)
    g_local = _plane_gravity(body, p0)
    t_impact = _ballistic_impact_time(state, target_height=target_height, g_local=g_local)
    atmosphere_top = float(drag.density_alts[-1]) if len(drag.density_alts) else 0.0
    horizon_max = max(
        config.horizon_factor * t_impact,
        2.0 * config.horizon_min,
    )
    tf_guess = max(config.horizon_min, t_impact)

    vz_up = max(float(v0[2]), 0.0)
    z_apex = float(p0[2]) + vz_up * vz_up / (2.0 * max(g_local, _BOUND_FLOOR))
    z_max = max(float(p0[2]), z_apex, float(target_height), atmosphere_top) * (
        1.0 + config.bounds_margin
    )
    t_go = max(t_impact, config.horizon_min)
    lateral_authority = float(engine.available_thrust) / m0
    if config.pointing_constraint:
        lateral_authority *= float(np.sin(np.radians(config.max_tilt_deg)))
    xy_limit = max(
        (
            float(np.max(np.abs(p0[:2])))
            + float(np.max(np.abs(v0[:2]))) * t_go
            + 0.5 * lateral_authority * t_go * t_go
        )
        * (1.0 + config.bounds_margin),
        _BOUND_FLOOR,
    )
    v_limit = (float(np.max(np.abs(v0))) + g_local * horizon_max) * (
        1.0 + config.velocity_margin
    )
    t_max = float(engine.available_thrust)
    t_min = config.min_throttle * t_max
    inertia = np.asarray(attitude.inertia, dtype=float)
    torque_limit = np.asarray(attitude.max_torque, dtype=float)

    # Quaternion `(x, y, z, w)` rotating body +y onto target +z (nose up).
    q_final = np.array([np.sin(np.pi / 4.0), 0.0, 0.0, np.cos(np.pi / 4.0)])
    q0 = np.array(
        [state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w],
        dtype=float,
    )
    q0 /= max(float(np.linalg.norm(q0)), 1e-12)
    if float(np.dot(q0, q_final)) < 0.0:
        q0 = -q0
    body_rate0 = _quat_to_matrix(q0).T @ np.asarray(
        [state.angular_velocity.x, state.angular_velocity.y, state.angular_velocity.z],
        dtype=float,
    )
    omega_limit = (
        np.abs(body_rate0) + torque_limit / inertia * horizon_max
    ) * (1.0 + config.bounds_margin)

    position = ox.State("position", shape=(3,))
    position.min = np.array([-xy_limit, -xy_limit, 0.0])
    position.max = np.array([xy_limit, xy_limit, z_max])
    position.initial = p0
    position.final = np.array([0.0, 0.0, float(target_height)])
    position.guess = np.linspace(p0, position.final, n)

    velocity = ox.State("velocity", shape=(3,))
    velocity.min = np.array([-v_limit, -v_limit, -v_limit])
    velocity.max = np.array([v_limit, v_limit, v_limit])
    if config.max_descent_speed is not None:
        velocity.min = np.array(
            [-v_limit, -v_limit, -float(config.max_descent_speed)]
        )
    velocity.initial = v0
    velocity.final = np.zeros(3)
    velocity.guess = np.linspace(v0, np.zeros(3), n)

    mass = ox.State("mass", shape=(1,))
    mass.min = np.array([float(engine.dry_mass)])
    mass.max = np.array([m0])
    mass.initial = np.array([m0])
    mass.final = [("maximize", float(engine.dry_mass))]
    mass.guess = np.full((n, 1), m0)

    attitude_state = ox.State("attitude", shape=(4,))
    attitude_state.min = -np.ones(4)
    attitude_state.max = np.ones(4)
    attitude_state.initial = q0
    attitude_state.final = q_final
    attitude_state.guess = np.tile(q0, (n, 1))

    angular = ox.State("angular_velocity", shape=(3,))
    angular.min = -omega_limit
    angular.max = omega_limit
    angular.initial = body_rate0
    angular.final = np.zeros(3)
    angular.guess = np.tile(body_rate0, (n, 1))

    thrust = ox.Control("thrust", shape=(1,))
    if config.free_ignition:
        # Burn-throttle fraction: the physical engine throttle is reconstructed
        # from this fraction and the smooth ignition gate.
        thrust.min = np.array([0.0])
        thrust.max = np.array([1.0])
        thrust.scaling_min = np.array([0.0])
        thrust.scaling_max = np.array([config.thrust_scaling])
        thrust.guess = np.full((n, 1), 0.6)
    else:
        thrust.min = np.array([t_min])
        thrust.max = np.array([t_max])
        thrust.scaling_min = np.array([t_min])
        thrust.scaling_max = np.array([max(config.thrust_scaling * t_max, t_min)])
        thrust.guess = np.full((n, 1), min(max(m0 * g_local, t_min), t_max))

    torque = ox.Control("torque", shape=(3,))
    torque.min = -torque_limit
    torque.max = torque_limit
    torque.scaling_min = -config.thrust_scaling * torque_limit
    torque.scaling_max = config.thrust_scaling * torque_limit
    torque.guess = np.zeros((n, 3))

    thrust_max = ox.Parameter("thrust_max", value=t_max)
    isp = ox.Parameter("isp", value=float(engine.isp))
    accel_bias = ox.Parameter("accel_bias", shape=(3,), value=np.zeros(3))
    inertia_param = ox.Parameter("inertia", shape=(3,), value=inertia)
    torque_max_param = ox.Parameter("torque_max", shape=(3,), value=torque_limit)

    time_axis = ox.Time(
        initial=0.0,
        final=ox.Free(tf_guess),
        min=0.0,
        max=horizon_max,
        guess=np.linspace(0.0, tf_guess, n).reshape(-1, 1),
        time_dilation_min=0.3 * tf_guess,
        time_dilation_max=3.0 * tf_guess,
    )

    ignition_time: Any = None
    if config.free_ignition:
        width = max(float(config.ignition_gate_width), 1e-3)
        ignition_max = (
            horizon_max
            if config.max_ignition_time is None
            else min(horizon_max, float(config.max_ignition_time))
        )
        if ignition_max < 0.0:
            raise ValueError(
                f"max_ignition_time must be non-negative, got {ignition_max}"
            )
        ignition_guess = min(max(float(config.ignition_guess), 0.0), ignition_max)
        ignition_time = ox.State("ignition_time", shape=(1,))
        ignition_time.min = np.array([0.0])
        ignition_time.max = np.array([ignition_max])
        ignition_time.initial = [ox.Free(ignition_guess)]
        ignition_time.final = [ox.Free(ignition_guess)]
        ignition_time.guess = np.full((n, 1), ignition_guess)
        gate = 1.0 / (
            1.0
            + ox.Exp(
                -(time_axis[0] - ignition_time[0]) / width
            )
        )
        effective_thrust = gate * (
            config.min_throttle * thrust_max
            + (1.0 - config.min_throttle) * thrust_max * thrust[0]
        )
    else:
        effective_thrust = thrust[0]

    center = np.asarray(body.body_center, dtype=float)
    omega = np.asarray(body.omega, dtype=float)
    d = position - center
    radius = ox.linalg.Norm(d)
    gravity = -body.mu * d / (radius**3)
    rotation = -2.0 * _cross(omega, velocity, ox) - _cross(
        omega, _cross(omega, d, ox), ox
    )

    altitude = radius - float(drag.sea_level_radius)
    table_top = float(drag.density_alts[-1])
    density = ox.Max(
        ox.Cinterp(ox.Min(ox.Max(altitude, 0.0), table_top), drag.density_alts, drag.density_vals),
        0.0,
    )
    speed = _smooth_norm(velocity, config.speed_norm_epsilon, ox)
    drag_accel = -0.5 * density * float(drag.cd_area) * speed * velocity / mass[0]

    # Body→inertial rotation; thrust acts along body +y.
    rotation_body = _quat_to_dcm_expr(attitude_state, ox)
    body_thrust = ox.Concat(0.0, effective_thrust, 0.0)
    thrust_inertial = rotation_body @ body_thrust

    w1 = angular[0]
    w2 = angular[1]
    w3 = angular[2]
    q1 = attitude_state[0]
    q2 = attitude_state[1]
    q3 = attitude_state[2]
    q4 = attitude_state[3]
    attitude_dot = ox.Concat(
        0.5 * (w1 * q4 - w2 * q3 + w3 * q2),
        0.5 * (w1 * q3 - w3 * q1 + w2 * q4),
        0.5 * (w2 * q1 - w1 * q2 + w3 * q4),
        -0.5 * (w1 * q1 + w2 * q2 + w3 * q3),
    )
    angular_momentum = ox.Concat(
        inertia_param[0] * w1,
        inertia_param[1] * w2,
        inertia_param[2] * w3,
    )
    gyroscopic = _cross(angular, angular_momentum, ox)
    angular_dot = ox.Concat(
        (torque[0] - gyroscopic[0]) / inertia_param[0],
        (torque[1] - gyroscopic[1]) / inertia_param[1],
        (torque[2] - gyroscopic[2]) / inertia_param[2],
    )

    dynamics = {
        "position": velocity,
        "velocity": thrust_inertial / mass[0] + gravity + rotation + drag_accel + accel_bias,
        "mass": -effective_thrust / (isp * G0),
        "attitude": attitude_dot,
        "angular_velocity": angular_dot,
    }
    if ignition_time is not None:
        dynamics["ignition_time"] = 0.0

    constraints: list[Any] = []
    state_components: tuple[Any, ...] = (position, velocity, mass, attitude_state, angular)
    if ignition_time is not None:
        state_components = state_components + (ignition_time,)
    for component in state_components:
        constraints.extend(
            [
                ox.ctcs(component <= component.max, idx=0),
                ox.ctcs(component.min <= component, idx=0),
            ]
        )
    norm_squared = ox.Sum(attitude_state * attitude_state)
    constraints.append(ox.ctcs(norm_squared <= 1.0 + config.quat_norm_tol, idx=0))
    constraints.append(ox.ctcs(1.0 - config.quat_norm_tol <= norm_squared, idx=0))
    if config.free_ignition:
        constraints.append(ox.ctcs(thrust[0] <= 1.0, idx=1))
        constraints.append(ox.ctcs(0.0 <= thrust[0], idx=1))
    else:
        constraints.append(ox.ctcs(thrust[0] <= thrust_max, idx=1))
        constraints.append(ox.ctcs(0.0 <= thrust[0], idx=1))
    constraints.append(ox.ctcs(-torque_max_param <= torque, idx=1))
    constraints.append(ox.ctcs(torque <= torque_max_param, idx=1))

    if config.pointing_constraint:
        cos_tilt = float(np.cos(np.radians(config.max_tilt_deg)))
        nose_inertial = rotation_body @ ox.Concat(0.0, 1.0, 0.0)
        constraints.append(ox.ctcs(cos_tilt <= nose_inertial[2], idx=2))

    tan_slope = float(np.tan(np.radians(config.glide_slope_deg)))
    scale = max(float(p0[2]), float(target_height), 1.0)
    lateral = (position[0] / scale) ** 2 + (position[1] / scale) ** 2
    height_above_site = (position[2] - float(target_height)) / scale
    constraints.append(
        ox.ctcs(lateral <= (tan_slope * height_above_site) ** 2, idx=3)
    )

    problem = ox.Problem(
        dynamics=dynamics,
        states=[position, velocity, mass, attitude_state, angular]
        + ([ignition_time] if ignition_time is not None else []),
        controls=[thrust, torque],
        time=time_axis,
        constraints=constraints,
        N=n,
        algorithm=_algorithm_config(config, ox),
        discretizer={"ode_solver": config.ode_solver},
        float_dtype=config.float_dtype,
    )
    problem.settings.dev.printing = False

    return DescentProblem(
        problem,
        position=position,
        velocity=velocity,
        mass=mass,
        thrust=thrust,
        n=n,
        config=config,
        g_local=g_local,
        target_height=target_height,
        time=time_axis,
        horizon_max=horizon_max,
        xy_limit=xy_limit,
        dof="6dof",
        attitude=attitude_state,
        angular_velocity=angular,
        torque=torque,
        attitude_final=q_final,
        ignition_time=ignition_time,
    )


@dataclass(frozen=True)
class _Refresh:
    state: FlightState
    engine: EngineModel
    tf_hint: float | None
    accel_bias: np.ndarray | None = None
    attitude: AttitudeModel | None = None


@dataclass
class _BoosterState:
    """Mutable per-booster bookkeeping owned by the solve service."""

    booster_id: str
    problem: DescentProblem
    config: ScvxConfig
    pending: _Refresh | None = None
    plan: Plan | None = None
    t0_ut: float = 0.0
    iterations: int = 0
    converged: bool = False
    solver_converged: bool = False
    plan_source: str = ""
    idle: bool = True
    last_error: str | None = None
    last_plan_time: float = 0.0
    in_queue: bool = False
    busy: bool = False


@dataclass(frozen=True)
class SolveStats:
    """Snapshot of one booster's solver state (for diagnostics/logging)."""

    iterations: int
    """SCP iterations spent on the current horizon."""

    converged: bool
    """Whether the current horizon converged or hit its iteration cap."""

    idle: bool
    """Whether the solver is waiting for the next :meth:`ScvxSolveService.refresh`."""

    last_error: str | None
    """Last solver failure (``None`` after a successful refresh)."""

    plan_age: float | None
    """Seconds since the last published plan, or ``None`` before the first."""

    queue_depth: int
    """Number of boosters waiting in the solve queue (including this one)."""

    solver_converged: bool = False
    """Whether the last SCP step reported actual solver convergence.

    ``converged`` above becomes true when the iteration cap is reached; this
    field distinguishes that from a genuine convergence flag.
    """

    plan_source: str = ""
    """Source of the last published plan (``multishot`` / ``nodal`` / ...)."""


class ScvxSolveService:
    """Multi-booster asynchronous SCP solve service.

    The service owns one :class:`DescentProblem` per booster and a work queue of
    *boosters*.  Refresh requests are per booster and latest-wins: only the
    newest measurement is kept, because a stale state is useless for control.
    Worker threads pop a booster, apply its pending refresh (warm-starting from
    its previous plan), run one SCP iteration and publish the iterate as a
    :class:`Plan`; the booster is re-queued until its iteration budget or
    convergence is reached.

    Each ``Problem`` is touched by at most one worker at a time, and the
    per-booster plan/stat slots are guarded by a lock, so the control threads
    can call :meth:`latest_plan` / :meth:`stats` at any time.

    ``workers`` defaults to 1.  The OpenSCvx cvxpy backend solves on the Python
    host, so extra workers mostly time-slice under the GIL; raise this only
    after verifying the solver's thread safety and measuring a gain.
    """

    def __init__(self, *, workers: int = 1) -> None:
        if workers < 1:
            raise ValueError(f"workers must be >= 1, got {workers}")
        self._workers = int(workers)
        self._states: dict[str, _BoosterState] = {}
        self._lock = threading.Lock()
        self._queue: queue.PriorityQueue[tuple[int, float, int, str]] = queue.PriorityQueue()
        self._seq = 0
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._initialized = False
        self._started = False
        self._float_dtype: str | None = None

    # -- registration / lifecycle -----------------------------------------
    def register(
        self,
        booster_id: str,
        problem: DescentProblem,
        *,
        config: ScvxConfig | None = None,
    ) -> None:
        """Register a booster's problem.

        Args:
            booster_id: Caller-chosen key (usually the ``ConnectionManager`` id).
            problem: The built OCP for this vessel.
            config: Solver policy; defaults to ``problem.config``.

        Raises:
            RuntimeError: if the service has already been initialized.
            ValueError: on a duplicate id or a mixed ``float_dtype``.
        """
        cfg = config if config is not None else problem.config
        with self._lock:
            if self._initialized:
                raise RuntimeError("cannot register a booster after initialize()")
            if booster_id in self._states:
                raise ValueError(f"booster {booster_id!r} is already registered")
            if self._float_dtype is None:
                self._float_dtype = cfg.float_dtype
            elif cfg.float_dtype != self._float_dtype:
                raise ValueError(
                    "all problems must share one float_dtype "
                    f"({self._float_dtype!r} != {cfg.float_dtype!r}); JAX's "
                    "precision setting is process-wide"
                )
            self._states[booster_id] = _BoosterState(
                booster_id=booster_id,
                problem=problem,
                config=cfg,
            )

    def initialize(self) -> None:
        """Compile every registered problem (serial; a few seconds each).

        The problems are initialized one at a time so JAX compilations do not
        compete for memory, and so the process-wide dtype is established by the
        first problem before the rest are lowered.
        """
        with self._lock:
            if self._initialized:
                return
            states = list(self._states.values())
        for state in states:
            state.problem.initialize()
        with self._lock:
            self._initialized = True

    def start(self) -> None:
        """Start the worker threads.

        Raises:
            RuntimeError: if :meth:`initialize` has not been called.
        """
        with self._lock:
            if not self._initialized:
                raise RuntimeError("call initialize() before start()")
            if self._started:
                return
            self._started = True
            self._stop.clear()
        for index in range(self._workers):
            thread = threading.Thread(
                target=self._run, name=f"scvx-worker-{index}", daemon=True
            )
            self._threads.append(thread)
            thread.start()

    def stop(self) -> None:
        """Stop the workers and wait for them to exit."""
        self._stop.set()
        for _ in range(self._workers):
            self._queue.put((-1, 0.0, -1, ""))
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()
        with self._lock:
            self._started = False

    # -- control-loop API --------------------------------------------------
    def refresh(
        self,
        booster_id: str,
        state: FlightState,
        *,
        engine: EngineModel,
        tf_hint: float | None = None,
        accel_bias: np.ndarray | None = None,
        attitude: AttitudeModel | None = None,
    ) -> None:
        """Request a new horizon for *booster_id* starting at *state*.

        Only the latest request per booster is kept; a slow solver simply skips
        stale measurements.  Safe to call from any control thread.

        Args:
            state: Fresh measurement; becomes the new initial condition.
            engine: Current engine-set capability.
            tf_hint: Optional time-of-flight hint (diagnostics/seed).
            accel_bias: Optional model-mismatch acceleration bias (m/s²).
            attitude: Optional inertia/torque envelope update for 6-DOF
                problems (ignored by point-mass problems).

        Raises:
            RuntimeError: if :meth:`initialize` has not been called.
            KeyError: if *booster_id* is not registered.
        """
        entry: tuple[int, float, int, str] | None = None
        with self._lock:
            if not self._initialized:
                raise RuntimeError("call initialize() before refresh()")
            state_slot = self._states.get(booster_id)
            if state_slot is None:
                raise KeyError(f"unknown booster id {booster_id!r}")
            state_slot.pending = _Refresh(
                state=state,
                engine=engine,
                tf_hint=tf_hint,
                accel_bias=accel_bias,
                attitude=attitude,
            )
            if not state_slot.in_queue and not state_slot.busy:
                state_slot.in_queue = True
                entry = (0, time.monotonic(), self._seq, booster_id)
                self._seq += 1
        if entry is not None:
            self._queue.put(entry)

    def latest_plan(self, booster_id: str) -> Plan | None:
        """Return the most recent published plan for *booster_id*."""
        with self._lock:
            state = self._states.get(booster_id)
            if state is None:
                raise KeyError(f"unknown booster id {booster_id!r}")
            return state.plan

    def last_error(self, booster_id: str) -> str | None:
        """Return the last solver error for *booster_id* (``None`` if healthy)."""
        with self._lock:
            state = self._states.get(booster_id)
            if state is None:
                raise KeyError(f"unknown booster id {booster_id!r}")
            return state.last_error

    def stats(self, booster_id: str) -> SolveStats:
        """Return a diagnostics snapshot for *booster_id*."""
        now = time.monotonic()
        with self._lock:
            state = self._states.get(booster_id)
            if state is None:
                raise KeyError(f"unknown booster id {booster_id!r}")
            age = (
                None
                if state.plan is None or state.last_plan_time <= 0.0
                else max(0.0, now - state.last_plan_time)
            )
            return SolveStats(
                iterations=state.iterations,
                converged=state.converged,
                idle=state.idle,
                last_error=state.last_error,
                plan_age=age,
                queue_depth=self._queue.qsize(),
                solver_converged=state.solver_converged,
                plan_source=state.plan_source,
            )

    @property
    def queue_depth(self) -> int:
        """Number of boosters currently waiting in the solve queue."""
        return int(self._queue.qsize())

    # -- worker ------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                priority, _, _, booster_id = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if priority < 0 or not booster_id:
                continue
            with self._lock:
                state = self._states.get(booster_id)
                if state is None:
                    continue
                state.in_queue = False
                if state.busy:
                    continue
                state.busy = True
                pending = state.pending
                state.pending = None
                previous_plan = state.plan
            if pending is not None:
                try:
                    self._apply_refresh(state, pending, previous_plan)
                except Exception as exc:  # noqa: BLE001 - keep the last plan
                    self._fail(state, exc)
                    continue
            if state.idle:
                self._release_idle(state)
                continue
            self._step(state)

    def _release_idle(self, state: _BoosterState) -> None:
        """Clear the busy flag and re-queue a refresh that arrived mid-flight."""
        entry: tuple[int, float, int, str] | None = None
        with self._lock:
            state.busy = False
            if (
                state.pending is not None
                and not state.in_queue
                and not self._stop.is_set()
            ):
                state.in_queue = True
                entry = (0, time.monotonic(), self._seq, state.booster_id)
                self._seq += 1
        if entry is not None:
            self._queue.put(entry)

    def _apply_refresh(
        self,
        state: _BoosterState,
        refresh: _Refresh,
        previous_plan: Plan | None,
    ) -> None:
        state.problem.set_engine(refresh.engine)
        if refresh.attitude is not None:
            state.problem.set_attitude_model(refresh.attitude)
        if refresh.accel_bias is not None:
            state.problem.set_accel_bias(refresh.accel_bias)
        state.problem.set_initial(
            refresh.state,
            previous=previous_plan,
            tf_hint=refresh.tf_hint,
        )
        state.problem.reset()
        with self._lock:
            state.t0_ut = float(refresh.state.ut)
            state.iterations = 0
            state.converged = False
            state.solver_converged = False
            state.idle = False
            state.last_error = None

    def _step(self, state: _BoosterState) -> None:
        try:
            result = state.problem.step()
            converged = bool(result.get("converged", False))
            plan = state.problem.read_plan(state.t0_ut, converged=converged)
        except Exception as exc:  # noqa: BLE001 - keep flying on the last plan
            self._fail(state, exc)
            return

        now = time.monotonic()
        requeue: tuple[int, float, int, str] | None = None
        with self._lock:
            state.plan = plan
            state.plan_source = getattr(state.problem, "last_plan_source", "")
            state.solver_converged = converged
            state.iterations += 1
            state.converged = converged or (
                state.iterations >= state.config.max_iters_per_refresh
            )
            state.last_plan_time = now
            state.busy = False
            if state.converged:
                state.idle = True
            if state.pending is not None and not state.in_queue and not self._stop.is_set():
                # A measurement arrived while this iteration was running:
                # service it before continuing the old horizon.
                state.in_queue = True
                requeue = (0, now, self._seq, state.booster_id)
                self._seq += 1
            elif not state.idle and not state.in_queue and not self._stop.is_set():
                state.in_queue = True
                requeue = (1, now, self._seq, state.booster_id)
                self._seq += 1
        if requeue is not None:
            self._queue.put(requeue)

    def _fail(self, state: _BoosterState, exc: BaseException) -> None:
        now = time.monotonic()
        requeue: tuple[int, float, int, str] | None = None
        with self._lock:
            state.last_error = f"{type(exc).__name__}: {exc}"
            state.solver_converged = False
            state.converged = True
            state.idle = True
            state.busy = False
            if state.pending is not None and not state.in_queue and not self._stop.is_set():
                state.in_queue = True
                requeue = (0, now, self._seq, state.booster_id)
                self._seq += 1
        if requeue is not None:
            self._queue.put(requeue)


class ScvxPlanner:
    """Single-vessel convenience wrapper around :class:`ScvxSolveService`.

    Kept for simple scripts and tests; multi-booster callers should use the
    service directly and address plans/stats by ``booster_id``.
    """

    _BOOSTER_ID = "default"

    def __init__(self, problem: DescentProblem, *, config: ScvxConfig | None = None) -> None:
        self._service = ScvxSolveService(workers=1)
        self._service.register(self._BOOSTER_ID, problem, config=config)

    def initialize(self) -> None:
        """Compile the OCP (a few seconds; call ahead of the descent)."""
        self._service.initialize()

    def start(self) -> None:
        """Start the worker thread."""
        self._service.start()

    def stop(self) -> None:
        """Stop the worker thread and wait for it to exit."""
        self._service.stop()

    def refresh(
        self,
        state: FlightState,
        *,
        engine: EngineModel,
        tf_hint: float | None = None,
        accel_bias: np.ndarray | None = None,
        attitude: AttitudeModel | None = None,
    ) -> None:
        """Request a new horizon starting at *state*."""
        self._service.refresh(
            self._BOOSTER_ID,
            state,
            engine=engine,
            tf_hint=tf_hint,
            accel_bias=accel_bias,
            attitude=attitude,
        )

    def latest_plan(self) -> Plan | None:
        """Return the most recent published plan (``None`` before the first)."""
        return self._service.latest_plan(self._BOOSTER_ID)

    @property
    def last_error(self) -> str | None:
        """Human-readable description of the last solver failure, if any."""
        return self._service.last_error(self._BOOSTER_ID)

    @property
    def iterations(self) -> int:
        """SCP iterations spent on the current horizon."""
        return self._service.stats(self._BOOSTER_ID).iterations

    @property
    def converged(self) -> bool:
        """Whether the current horizon has converged (or hit its iteration cap)."""
        return self._service.stats(self._BOOSTER_ID).converged
