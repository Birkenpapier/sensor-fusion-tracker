# ADR-0001: Linear Kalman filter with a constant-velocity model

**Status:** Accepted · **Date:** 2026-06-07

## Context

We fuse two sensors that both report a Cartesian position `(x, y)` of a moving
target. We need a recursive estimator that (a) smooths sensor noise, (b) fills
the gaps between asynchronous, different-rate sensors, and (c) is small enough
to build and fully test in a weekend.

The candidate filters:

- **Linear Kalman filter (KF)** — optimal for linear models with Gaussian noise.
- **Extended KF (EKF)** — needed only if the measurement or motion model is
  nonlinear (e.g. radar reporting range/azimuth).
- **IMM / particle filters** — for strong manoeuvres or non-Gaussian noise.

## Decision

Use a **linear KF** with a **constant-velocity (CV)** motion model, state
`[px, py, vx, vy]`. Because both sensors are modelled as reporting Cartesian
position, the observation model `H` is linear, so the plain KF is exact — no
Jacobians, no linearisation error. Sensor differences are captured entirely in
each sensor's measurement-noise matrix `R`.

Process noise uses the standard discrete white-noise-acceleration model so the
filter tolerates the gentle turn in the simulated trajectory without diverging.

## Consequences

- **+** Minimal, provably-correct, easy to unit-test.
- **+** Adding a third position sensor is just another `R`; no core change.
- **−** A hard manoeuvre will lag (CV assumes constant velocity). Acceptable for
  the demo; documented as an extension (EKF/IMM).
- **−** Modelling the radar in true polar coordinates would be more realistic;
  deferred to the EKF extension to keep the core linear and testable.
