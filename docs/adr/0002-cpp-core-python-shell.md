# ADR-0002: C++ estimation core, Python simulation/visualisation, CSV seam

**Status:** Accepted · **Date:** 2026-06-07

## Context

The project has three concerns: generating realistic test data, the real-time
estimation algorithm, and scoring/plotting results. They have very different
constraints — the estimator must be fast and deterministic; the sim and
analysis benefit from a rich numerical/plotting ecosystem.

## Decision

Split by language along those seams:

- **C++** owns the fusion core (`sft_app`). It is the component that would run
  on the embedded/real-time side in a real system, so it carries no scripting
  dependency and allocates nothing on the per-measurement path.
- **Python** owns simulation (`simulate.py`) and scoring/visualisation
  (`visualize.py`), using NumPy/pandas/matplotlib.
- **CSV files** are the interface between them: `measurements.csv` in,
  `track.csv` out.

## Consequences

- **+** Clean, inspectable boundary; either side can be swapped or driven
  independently (e.g. feed `sft_app` real recorded data).
- **+** Demonstrates all three target languages doing what each is best at.
- **+** The C++ core stays dependency-light and portable to a constrained target.
- **−** CSV is not a streaming/real-time transport. For an actual real-time node
  the seam would become a socket or shared-memory ring buffer; noted as an
  extension. The algorithm code would not change — only the I/O at the edges.
- **−** Two toolchains to set up (CMake + pip). Mitigated by `run_demo.sh`.
