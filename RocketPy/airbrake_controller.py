"""
Airbrake deployment controller for RocketPy simulation.

Mirrors the two-stage state machine in the H5 embedded firmware:

    flight_trigger.h / flight_trigger.c

State machine:
    IDLE  ──(az >  arm_g)──▶  ARMED
    ARMED ──(az < fire_g)──▶  DEPLOYED

    DEPLOYED is a terminal state (one-shot, no retraction).

Both thresholds use the SIGNED vertical (z-axis) inertial acceleration, not
the magnitude.  Using magnitude is ambiguous: it crosses the same value both
at motor ignition (rising) and at burnout (falling), so a magnitude < threshold
check can fire during the early-flight transient instead of at burnout.

Signed-acceleration semantics:
  • During motor burn:  az ≈ +10 g  (thrust >> gravity + drag)
  • At burnout:         az crosses 0 and goes negative  (gravity + drag >> 0 thrust)
  • arm_g  > 0  → arms once the motor has built up thrust  (az rising past arm_g)
  • fire_g ≤ 0  → deploys the moment the net vertical acceleration goes negative
                  (i.e., right at burnout, not near apogee when drag has bled off)

Once DEPLOYED, the airbrakes stay at full deployment (80°) for the rest of
the flight.  This exactly matches the embedded lockout:

    "Once fired the trigger locks out so the sequence is only started
     once per power cycle."

Usage in mags_test_launch.py
─────────────────────────────
    from airbrake_controller import AirbrakeController

    ctrl = AirbrakeController()          # one instance per Monte Carlo run
    rocket.add_air_brakes(
        drag_coefficient_curve = airbrake_model.drag_coefficient_curve,
        controller_function    = ctrl,
        sampling_rate          = 50,
        reference_area         = airbrake_model.reference_area,
        clamp                  = True,
    )

Call ctrl.reset() between Monte Carlo runs to re-arm the state machine.
"""

import numpy as np

_G_SI = 9.80665          # m/s² per standard gravity


class AirbrakeController:
    """RocketPy controller that mirrors the H5 firmware flight trigger.

    Parameters
    ----------
    max_angle : float
        Full deployment angle in degrees.  Maps to deployment_level = 1.0.
        Must match AIRBRAKE_MAX_ANGLE in mags_test_launch.py (default 80°).
    arm_g : float
        Acceleration magnitude threshold [g] to enter ARMED state.
        Mirrors FLIGHT_TRIGGER_ARM_G in flight_trigger.h (default 5.0 g).
    fire_g : float
        Acceleration magnitude threshold [g] that triggers deployment once
        ARMED.  Mirrors FLIGHT_TRIGGER_FIRE_G in flight_trigger.h (default
        3.0 g).
    """

    # State constants — match H5 FlightTriggerState_t enum names
    _IDLE     = "IDLE"
    _ARMED    = "ARMED"
    _DEPLOYED = "DEPLOYED"   # equivalent to FLIGHT_TRIGGER_FIRED

    def __init__(self, max_angle=80.0, arm_g=4.0, fire_g=0.0):
        self.max_angle  = float(max_angle)
        self.arm_g      = float(arm_g)
        self.fire_g     = float(fire_g)
        self._state     = self._IDLE
        self._prev_vel  = None   # velocity [vx,vy,vz] at previous controller call

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def reset(self):
        """Reset to IDLE — call before each Monte Carlo run."""
        self._state    = self._IDLE
        self._prev_vel = None

    @property
    def state(self):
        """Current state name string: 'IDLE', 'ARMED', or 'DEPLOYED'."""
        return self._state

    @property
    def is_deployed(self):
        """True once deployment has been triggered."""
        return self._state == self._DEPLOYED

    # ------------------------------------------------------------------
    # Acceleration estimation
    # ------------------------------------------------------------------

    def _accel_signed_g(self, state_vector, sampling_rate):
        """Estimate signed vertical (z-axis) inertial acceleration [g].

        Uses finite-difference of the vertical velocity component (state[5] = vz).
        dt = 1 / sampling_rate matches the fixed controller call interval exactly.

        Sign convention:
          +g  during motor burn   (thrust accelerates rocket upward)
           0  at burnout          (net force crosses zero)
          -g  during coast        (gravity + drag decelerate the rocket)

        Returning the signed z-component — not the magnitude — makes the two
        events unambiguous:
          motor ignition : az rises from ~0 to +10 g  → triggers ARMED
          burnout        : az falls from +10 g through 0 → triggers DEPLOYED

        Returns 0.0 on the first call (no previous sample available yet).

        State vector layout: [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]
        """
        vz_now = state_vector[5]

        if self._prev_vel is None:
            self._prev_vel = (state_vector[3], state_vector[4], vz_now)
            return 0.0

        dt = 1.0 / max(float(sampling_rate), 1e-9)
        az = (vz_now - self._prev_vel[2]) / dt
        self._prev_vel = (state_vector[3], state_vector[4], vz_now)

        return az / _G_SI

    # ------------------------------------------------------------------
    # RocketPy controller_function interface
    # ------------------------------------------------------------------

    def __call__(
        self,
        time,
        sampling_rate,
        state_vector,
        _state_history,      # required by RocketPy 6-param signature; not used
        _observed_variables, # required by RocketPy 6-param signature; not used
        interactive_objects,
    ):
        """RocketPy controller callback (6-parameter form).

        RocketPy accepts 6, 7, or 8 explicit parameters — *args is not
        allowed because it inspects the signature to determine arity.

        Parameters
        ----------
        time : float
            Current simulation time [s].
        sampling_rate : float
            Controller call rate [Hz].
        state_vector : list
            [x, y, z, vx, vy, vz, e0, e1, e2, e3, wx, wy, wz]
        _state_history : list
            All previous state vectors (unused — we track velocity internally).
        _observed_variables : list
            Accumulated return values from prior controller calls (unused).
        interactive_objects : AirBrakes
            The AirBrakes object whose deployment_level we set.
        """
        air_brakes = interactive_objects

        # ── Terminal state: stay fully deployed ──────────────────────────
        if self._state == self._DEPLOYED:
            air_brakes.deployment_level = 1.0
            return

        accel_g = self._accel_signed_g(state_vector, sampling_rate)

        # ── IDLE → ARMED (motor burn detected) ───────────────────────────
        # az > arm_g: signed vertical accel above threshold means the motor
        # is pushing the rocket upward (typically +8..+15 g during burn).
        if self._state == self._IDLE:
            if accel_g > self.arm_g:
                self._state = self._ARMED
                print(
                    f"[TRIGGER] ARMED  — az = {accel_g:+.2f} g "
                    f"(arm threshold {self.arm_g:+.1f} g)  t = {time:.3f} s"
                )

        # ── ARMED → DEPLOYED (burnout detected) ──────────────────────────
        # az < fire_g (≤ 0): signed vertical accel has crossed zero and gone
        # negative — thrust is gone, gravity + drag now decelerate the rocket.
        if self._state == self._ARMED:
            if accel_g < self.fire_g:
                self._state = self._DEPLOYED
                air_brakes.deployment_level = 1.0
                print(
                    f"[TRIGGER] FIRED  — az = {accel_g:+.2f} g "
                    f"(fire threshold {self.fire_g:+.1f} g)  t = {time:.3f} s  "
                    f"→ deploying to {self.max_angle:.0f}°"
                )
                return

        # ── Not yet deployed: keep retracted ─────────────────────────────
        air_brakes.deployment_level = 0.0
