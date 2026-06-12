# ADR-0003: Joseph-form covariance update; dependencies via FetchContent

**Status:** Accepted · **Date:** 2026-06-07

## Context

Two smaller decisions worth recording.

### Covariance update form

The Kalman covariance update can be written as the short `P = (I - K H) P` or
the longer **Joseph form** `P = (I - K H) P (I - K H)ᵀ + K R Kᵀ`. The short
form is cheaper but, under floating-point round-off, can drift into a
non-symmetric or non-positive-definite `P`, which eventually breaks the filter
— a real failure mode for an estimator that runs for hours.

### Third-party dependencies

We need a linear-algebra library (Eigen) and a test framework (GoogleTest)
without vendoring source or requiring a manual system install.

## Decision

- Use the **Joseph form** for the covariance update. The extra cost is
  negligible at this state size and it guarantees symmetry/PD numerically.
- Pull **Eigen 3.4** and **GoogleTest 1.14** with CMake **FetchContent**, pinned
  to explicit tags and shallow-cloned.

## Consequences

- **+** Numerically robust filter; symmetry is even asserted in a unit test.
- **+** `git clone` + `cmake` builds with no manual dependency steps.
- **+** Pinned tags = reproducible builds.
- **−** First configure needs network access to fetch the deps (cached after).
- **−** Slightly more compute per update than the short form — immaterial here.
