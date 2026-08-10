# Understand This Project — From Zero

> You built a **sensor-fusion tracker**. This document explains it from scratch:
> the **what**, the **why**, the **how**, and the **where** (which file does what).
> Read it top to bottom once. By the end you'll be able to teach it back — which
> is exactly what you need to do in the interview.
>
> No prior knowledge of Kalman filters assumed. You know C++ and Python; that's enough.

---

## 0. The one-sentence summary

> **We have a moving target and two cheap, noisy sensors watching it. Each sensor
> is bad in a different way. We wrote a program that mathematically blends their
> readings over time into one accurate track — better than either sensor alone.**

That blending technique is a **Kalman filter**. The act of combining multiple
sensors is **sensor fusion**. That's the whole project. Everything below is just
*how* and *why*.

---

## 1. The problem, in plain words

Imagine a vehicle (or drone, or aircraft) moving across a field. You want to know
**where it is** and **how fast it's going**, at every moment. You can't measure
its true position directly — you only have sensors, and sensors lie a little.

We have two:

| Sensor | Good at | Bad at | How often it reports |
|---|---|---|---|
| **Radar** | measuring the **x** direction (range) — precise | the **y** direction — very noisy | 4 times per second |
| **EO/IR camera** | measuring the **y** direction (angle) — precise | the **x** direction — very noisy | 12 times per second |

This is **realistic**. Real radars are great at range but poor at cross-range
angle. Cameras are the opposite — great at angle, poor at depth. They are
**complementary**: each is strong exactly where the other is weak.

**The challenge:** neither sensor alone gives you a good position. Look at a
single radar reading — the x is trustworthy but the y could be off by 12 metres.
A single camera reading — the y is great but x could be off by 12 metres.

**The goal:** combine them so you take the *good* x from radar and the *good* y
from the camera — automatically, every frame, without being told which is which.
And smooth out the random noise using the fact that the target can't teleport
(it has momentum).

That's what the program does. Our result: each sensor alone is off by ~11–12 m;
**fused, we're off by ~1.1 m**. A ~90% improvement, from *math*, with the same
cheap sensors.

---

## 2. Why this is the "situational awareness" problem in defense

This exact problem — *fuse multiple imperfect sensors into one trusted track* —
is the core of:

- Ground-vehicle situational awareness (Rheinmetall / KNDS battle vehicles)
- Air-surveillance and counter-drone systems
- Naval / fire-control tracking
- Self-driving cars (lidar + radar + camera fusion)
- Aerospace navigation (GPS + inertial sensors — same math)

A defense interviewer sees "Kalman filter + sensor fusion" and immediately knows
you've touched the real thing. That's why we picked it.

> **Ethics note:** we only *track* and *estimate position* on synthetic data.
> There is no targeting, no weapon, no guidance. It's situational awareness —
> the defensive, "know what's around you" half of the problem.

---

## 3. The big idea: a "predict, then correct" loop

A Kalman filter is a loop with two steps that repeat forever:

```
   ┌──────────────────────────────────────────────────────────┐
   │                                                            │
   │   PREDICT  ──────────────▶   UPDATE   ─────────────────┐   │
   │  "where do I think the      "a sensor just reported.   │   │
   │   target moved to, based     blend my prediction with  │   │
   │   on its last known          the reading, trusting     │   │
   │   velocity?"                 whichever is more certain" │   │
   │                                                        │   │
   └────────────────◀───────────────────────────────────────┘   │
                  (the corrected estimate becomes                │
                   the starting point for the next predict)      │
```

### The intuition with a story

You're tracking a friend walking across a dark field. Every second you do this:

1. **PREDICT.** "Last I knew, they were *here* walking *that way* at *this speed*.
   One second passed, so they're probably *over there* now." You moved your guess
   forward using their **velocity**. This is dead-reckoning. But you're not sure —
   they might have sped up or turned — so your *uncertainty grows*.

2. **UPDATE.** Someone shines a weak flashlight and you glimpse them. The glimpse
   is fuzzy (noisy measurement). Now you have **two opinions**: your prediction,
   and the glimpse. You blend them. **Crucially, you weight the blend by how much
   you trust each one.** If the glimpse was clear and your prediction was shaky,
   you lean on the glimpse. If the glimpse was barely visible but you were
   confident in your prediction, you lean on the prediction.

That weighting — *how much to trust the new measurement vs. your prediction* —
is the single most important number in the whole filter. It's called the
**Kalman gain**. The genius of the Kalman filter is that it computes this optimal
weight automatically, every step, from the uncertainties.

### How fusion falls out for free

Here's the beautiful part. We don't tell the filter "take x from radar, y from
camera." We just tell it **how much each sensor can be trusted in each direction**
(radar: trust x, distrust y; camera: distrust x, trust y). The Kalman gain math
then *automatically* pulls x mostly from the radar readings and y mostly from the
camera readings. Fusion is an emergent property of "weight by trust." That's the
"aha" you want to be able to say out loud.

---

## 4. The math — gently, with what each symbol means

Don't panic at the matrices. Every one of them is just "a list of numbers that
describes the target or a sensor." Let's name them.

### 4.1 The state — what we're tracking

We describe the target with **4 numbers**, stacked in a column called the
**state vector** `x`:

```
x = [ px ]   position in x  (metres)
    [ py ]   position in y  (metres)
    [ vx ]   velocity in x  (m/s)
    [ vy ]   velocity in y  (m/s)
```

That's it. "Where it is" + "how fast it's going." Notice velocity is in there
even though no sensor measures velocity directly — the filter *infers* it from
how the position changes. (That inference is one of the slickest things it does.)

### 4.2 The covariance — how unsure we are

For every estimate we also keep a **4×4 matrix `P`** called the **covariance**.
Think of it as "the error bars on the state, and how the errors relate."

- The diagonal of `P` = how uncertain we are about each of px, py, vx, vy.
- Big numbers on the diagonal = "I have no idea." Small numbers = "I'm confident."

`P` shrinks when we get a measurement (we learned something) and grows when we
predict forward in time (the future is uncertain). Watching `P` shrink is
literally the filter becoming confident.

### 4.3 The motion model `F` — physics of "predict"

How does the target move in one time-step `dt`? With constant velocity:

```
new px = px + vx · dt      (you move by speed × time)
new py = py + vy · dt
new vx = vx                (velocity unchanged — our assumption)
new vy = vy
```

Written as a matrix `F` that we multiply the state by:

```
F = [ 1  0  dt 0 ]      so   x_predicted = F · x
    [ 0  1  0  dt]
    [ 0  0  1  0 ]
    [ 0  0  0  1 ]
```

Multiply it out and you get exactly the four lines above. This is the **constant-
velocity (CV) model**. "Assume it keeps going the way it was going."

### 4.4 Process noise `Q` — admitting the model is imperfect

The target doesn't *really* move at perfectly constant velocity — it can
accelerate or turn. `Q` is a matrix that says "here's how much the truth can
drift away from my constant-velocity assumption per step." It's how the filter
stays humble and doesn't over-trust its own prediction. Bigger `Q` = "expect more
manoeuvring."

### 4.5 The measurement model `H` — what a sensor sees

Our sensors report **position only** (x and y), not velocity. `H` is the matrix
that picks the position out of the full state:

```
H = [ 1 0 0 0 ]    so   H · x = [ px ]   (just the position)
    [ 0 1 0 0 ]                 [ py ]
```

### 4.6 Measurement noise `R` — how much a sensor lies

This is where the two sensors differ, and where fusion lives. `R` is a 2×2 matrix
of each sensor's noise:

```
Radar:  R = [ 1.5² = 2.25     0      ]   small x-noise, HUGE y-noise
            [   0          12² = 144 ]

Camera: R = [ 12² = 144      0       ]   HUGE x-noise, small y-noise
            [   0          1.5² = 2.25]
```

The filter reads `R` and thinks "for a radar reading, I can trust x but the y is
nearly worthless; for a camera reading, the opposite." **This is the only place
we encode the sensors' personalities.** Everything else is generic.

### 4.7 The two steps as equations

**PREDICT** (move the state and its uncertainty forward):

```
x = F · x                    move the estimate forward by velocity
P = F · P · Fᵀ + Q           grow the uncertainty (and add model humility Q)
```

**UPDATE** (fold in a sensor reading `z`):

```
y = z − H · x                "innovation": how wrong was my prediction? (measurement minus expectation)
S = H · P · Hᵀ + R           total uncertainty of that disagreement
K = P · Hᵀ · S⁻¹             THE KALMAN GAIN — how much to trust the measurement
x = x + K · y                nudge the estimate toward the measurement, scaled by K
P = (I − K·H) P (I−K·H)ᵀ + K·R·Kᵀ    shrink the uncertainty (we learned something)
```

### 4.8 The Kalman gain `K` — the heart of everything

Look at `K = P · Hᵀ · S⁻¹`. In words:

> "My own uncertainty (`P`) divided by the total uncertainty (`S`, which includes
> the sensor's noise `R`)."

- If **my prediction is shaky** (`P` big) and the **sensor is precise** (`R` small):
  `K` ≈ 1 → "trust the measurement, move most of the way toward it."
- If **my prediction is solid** (`P` small) and the **sensor is noisy** (`R` big):
  `K` ≈ 0 → "ignore the measurement, keep my prediction."

And because `P`, `R`, `K` are *matrices*, this trust is computed **separately for
x and y**. A radar reading has a near-1 gain on x and near-0 gain on y. A camera
reading is the reverse. **That's the fusion, done automatically.** This is the
sentence to memorize for the interview.

---

## 5. The code — where each idea lives

Now we map the math to the actual files. Here's the project, file by file.

```
sensor-fusion-tracker/
├── cpp/                         ← the C++ "real-time core" (the estimator)
│   ├── include/sft/
│   │   ├── types.hpp            ← data structures (Measurement, StateEstimate)
│   │   ├── kalman_filter.hpp    ← the generic Kalman filter (predict/update)  ★ the heart
│   │   └── tracker.hpp          ← config + the constant-velocity wrapper
│   ├── src/
│   │   ├── tracker.cpp          ← builds F, Q, R; runs the predict/update loop
│   │   └── main.cpp             ← reads/writes CSV, calls the tracker
│   └── tests/
│       └── test_kalman.cpp      ← GoogleTest proofs that it works
├── python/                      ← the "lab" around the core
│   ├── simulate.py              ← invents the truth + fake noisy sensor readings
│   ├── visualize.py             ← scores the result (RMSE) + draws the plot
│   └── requirements.txt
├── scripts/run_demo.sh          ← runs the whole thing with one command
├── docs/adr/                    ← Architecture Decision Records (why we chose X)
├── CMakeLists.txt               ← the C++ build recipe
└── README.md                    ← the public-facing summary
```

### 5.1 `cpp/include/sft/types.hpp` — the vocabulary

Plain structs. A `Measurement` is one sensor reading: `{ time, which sensor,
x, y }`. A `StateEstimate` is one filter output: `{ time, px, py, vx, vy, how-
sure }`. Nothing clever — just the nouns the rest of the code talks about.

### 5.2 `cpp/include/sft/kalman_filter.hpp` — the engine ★

This is the **generic** Kalman filter — it knows nothing about radars or
velocity. It just implements §4.7's equations. Two methods:

- `predict(F, Q)` → runs the two predict equations. Literally:
  ```cpp
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
  ```
  Compare to §4.7. It's a one-to-one translation.

- `update(z, H, R)` → runs the update equations:
  ```cpp
  const MeasVec  y = z - H * x_;                 // innovation  (§4.7)
  const MeasMat  S = H * P_ * H.transpose() + R; // disagreement uncertainty
  const GainMat  K = P_ * H.transpose() * S.inverse();  // Kalman gain
  x_ = x_ + K * y;                               // nudge toward measurement
  // Joseph form for the covariance (numerically stable):
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  ```

Two C++ details worth knowing (interviewers love these):

- **`template <int N, int M>`** — the filter is templated on state size `N` and
  measurement size `M`. Because the sizes are known at *compile time*, the matrix
  library (Eigen) can put everything on the **stack** — no `new`/`delete`, no heap
  allocation while running. That's what you want in a real-time system that can't
  afford unpredictable pauses. Ours is `KalmanFilter<4, 2>` (4 state, 2 measured).

- **Joseph form** — the covariance update is written the long way
  `(I−KH)P(I−KH)ᵀ + KRKᵀ` instead of the short `(I−KH)P`. Mathematically equal,
  but the long form stays *symmetric and valid* even after millions of
  floating-point round-offs. The short form can slowly corrupt `P` and break a
  filter that runs for hours. **This is a "I've done this for real" detail** —
  see [docs/adr/0003](docs/adr/0003-joseph-form-and-deps.md).

### 5.3 `cpp/include/sft/tracker.hpp` + `tracker.cpp` — the brain that knows our problem

The generic filter doesn't know it's tracking a constant-velocity target with a
radar and a camera. The **Tracker** supplies that knowledge:

- `transition(dt)` builds the `F` matrix from §4.3 for the actual time gap.
- `process_noise(dt)` builds `Q` from §4.4.
- `measurement_noise(sensor)` returns the radar `R` or the camera `R` from §4.6 —
  **this is the line that makes it "fusion."**
- `step(measurement)` is the loop body:
  1. If this is the very first measurement, **initialize**: put the state at that
     position, velocity = 0, and a *big* `P` ("I'm very unsure for now").
  2. Otherwise compute `dt` since the last measurement, **predict** forward by
     `dt`, then **update** with this sensor's `R`.
- `run(measurements)` just calls `step` for every measurement in time order and
  collects the estimates.

The **multi-rate** handling is subtle and worth noticing: radar and camera arrive
at different times, all merged into one time-sorted list. For each one we predict
forward by however long it's been since the *previous* measurement (could be from
either sensor), then update. The filter doesn't care which sensor is next — it
just predicts to that timestamp and folds in the reading with the right `R`.

### 5.4 `cpp/src/main.cpp` — the plumbing

No math here. It:
1. reads `measurements.csv` into a `vector<Measurement>`,
2. runs the `Tracker`,
3. writes the estimates to `track.csv`.

CSV is deliberately the boundary between C++ and Python — see
[docs/adr/0002](docs/adr/0002-cpp-core-python-shell.md). The C++ core stays a
clean, dependency-light thing that could later read from a real radar instead of
a file, without touching the algorithm.

### 5.5 `python/simulate.py` — the fake world

To test a tracker you need **ground truth** (the real path) so you can measure
error. In the real world you never have that, so we *invent* it:

- `trajectory(t)` defines the true path: a straight north-east leg, then a gentle
  turn. (The turn matters — it's where a constant-velocity filter is challenged,
  proving ours copes.)
- Then it **samples** that truth through each sensor, adding random Gaussian noise
  with the sensor's standard deviations (radar: ±1.5 m in x, ±12 m in y; camera:
  reverse). Radar at 4 Hz, camera at 12 Hz.
- Writes `truth.csv` (the secret real path, for scoring) and `measurements.csv`
  (the noisy readings, all the tracker is allowed to see).

The `--seed` makes the randomness reproducible — same run every time.

### 5.6 `python/visualize.py` — the judge

Reads the truth, the raw measurements, and the fused track, then:

- Computes **RMSE** (root-mean-square error — the typical distance from truth, in
  metres) for raw radar, raw camera, and the fused track.
- Prints the comparison (this is where you saw `radar 10.9, EO 12.0, fused 1.14`).
- Draws `fusion.png`: left = the trajectory (scattered sensor dots, black truth
  line, red fused track); right = fused error over time vs the sensors' RMSE.

### 5.7 `cpp/tests/test_kalman.cpp` — the proof

Four GoogleTest cases that *prove* the filter behaves, without eyeballing a plot:

1. **Prediction moves position by velocity × dt** (the physics is right).
2. **An update pulls the estimate toward the measurement and shrinks uncertainty**
   (learning works).
3. **The covariance stays symmetric** (the Joseph form does its job).
4. **End-to-end: the fused track has lower error than the raw measurements**
   (the whole point holds).

Being able to say "and it's covered by unit tests, including one that asserts
fusion actually beats the raw sensors" is a strong signal.

### 5.8 `CMakeLists.txt` — the build

Standard modern CMake. The one nice trick: **FetchContent** automatically
downloads the two dependencies — **Eigen** (the matrix math library) and
**GoogleTest** (the test framework) — pinned to specific versions, at build time.
So anyone can `git clone` and `cmake` with zero manual install steps.

---

## 6. The journey of a single measurement (tie it all together)

Follow one radar reading through the whole system:

1. **`simulate.py`** computes the true target position at t = 5.00 s, say
   `(true_x=32.8, true_y=22.9)`. It adds noise → radar reports `(33.1, 35.0)`.
   Note the y is way off (35 vs 22.9) — that's the radar's bad axis. Written to
   `measurements.csv`.
2. **`main.cpp`** reads that row into a `Measurement{ t=5.0, Radar, 33.1, 35.0 }`.
3. **`tracker.cpp` → step()**: it's been 0.083 s since the last reading, so it
   **predicts** the state forward 0.083 s (target glides along its known velocity).
   `P` grows a little.
4. It picks the **radar `R`** (trust x, distrust y) and calls the filter's
   **update**. The Kalman gain comes out high for x, near-zero for y. So the
   estimate's x moves toward 33.1, but its y **barely budges** toward the bogus 35
   — because the filter knows radar-y is garbage and it already had a good y from
   recent camera readings.
5. The corrected `(px, py, vx, vy)` is written to `track.csv`.
6. **`visualize.py`** later compares that px,py to the true (32.8, 22.9) and finds
   it's within ~1 m — even though this single radar reading was 12 m off in y.

That step 4 — *high gain on the good axis, near-zero on the bad axis* — is the
entire value of the project, happening 640 times.

---

## 7. How to run it and read the output

```bash
cd sensor-fusion-tracker
./scripts/run_demo.sh
```

This runs: install Python deps (into a `.venv`) → simulate → build C++ → fuse →
score + plot. You'll see:

```
position RMSE (metres):
  raw radar : 10.912
  raw EO    : 12.047
  fused     :  1.140
  fusion improves on the best single sensor by 89.6%
```

**How to read that:** on average, a raw radar fix is ~11 m from the truth, a raw
camera fix ~12 m. The fused track is ~1.1 m from truth. We turned two ~11 m
sensors into one ~1 m track. Open `data/fusion.png` to *see* it: the red line
threads right through the middle of the noisy dot-cloud, following the black
truth line even around the turn.

To run just the tests:

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
```

---

## 8. How to talk about it in the interview

When they say **"tell me about a project"**, say roughly this:

> "I built a multi-sensor fusion tracker in C++. The scenario: two complementary
> sensors watch a moving target — a radar that's precise in range but noisy
> cross-range, and an EO camera that's the opposite. Each alone has about 11–12 m
> error. I wrote a Kalman filter with a constant-velocity model that fuses them,
> and it gets the track down to about 1 m — a 90% improvement.
>
> The core is a templated, compile-time-sized Kalman filter so there's no heap
> allocation on the real-time path, and I used the Joseph-form covariance update
> for numerical stability over long runs. The clever part is that I never
> hard-code 'take x from radar, y from camera' — I just give each sensor its noise
> covariance, and the Kalman gain automatically weights each axis by trust. Fusion
> emerges from the math. It's covered by unit tests, including one that asserts the
> fused error actually beats the raw sensors."

Then expect these follow-ups (short answers):

- **"What is a Kalman filter?"** → "A recursive estimator that loops predict and
  update. Predict moves the state forward with a motion model and grows
  uncertainty; update blends in a measurement, weighted by the relative
  uncertainty of the prediction vs. the sensor. Optimal for linear systems with
  Gaussian noise."
- **"What's the Kalman gain?"** → "The weight that decides how much to trust the
  new measurement vs. the prediction. It's my uncertainty over total uncertainty;
  high when I'm unsure and the sensor is precise, low when the reverse."
- **"Why constant velocity?"** → "Simplest model sufficient for position-only
  fixes, and fully testable. For hard manoeuvres I'd move to an IMM or add
  acceleration; for a radar in true polar coordinates I'd need an EKF — that's the
  documented next step." (See the ADRs — you literally wrote these decisions down.)
- **"Why C++ and Python both?"** → "C++ for the deterministic real-time estimator,
  no scripting deps; Python for simulation and analysis where NumPy/matplotlib
  shine. CSV is the seam." (ADR-0002.)
- **"What's `R` and `Q`?"** → "`R` is per-sensor measurement noise — how much each
  sensor lies. `Q` is process noise — how much the target can deviate from my
  constant-velocity assumption."

---

## 9. Glossary (one line each)

- **State `x`** — the 4 numbers we track: px, py, vx, vy.
- **Covariance `P`** — our uncertainty about the state (error bars).
- **Motion model `F`** — how the state moves forward one step (constant velocity).
- **Process noise `Q`** — how much we admit the motion model is imperfect.
- **Measurement model `H`** — maps state to what a sensor sees (here: position only).
- **Measurement noise `R`** — how noisy each sensor is (the per-sensor "trust").
- **Innovation `y`** — measurement minus prediction; "how surprised am I?"
- **Kalman gain `K`** — how much to trust the measurement vs. the prediction.
- **Predict** — step the state and grow uncertainty.
- **Update** — fold in a measurement and shrink uncertainty.
- **RMSE** — root-mean-square error; average distance from truth, in metres.
- **EKF** — Extended Kalman Filter; the version for *nonlinear* models (next step).
- **Sensor fusion** — combining multiple sensors into one better estimate.

---

## 10. If you want to truly own it — do these in order

1. **Run it**, then open `data/fusion.png`. Watch the red line beat the dots.
2. Open `kalman_filter.hpp`. Read `predict()` and `update()` with §4.7 beside it.
   Confirm each line matches an equation. That's the whole algorithm — ~10 lines.
3. In `simulate.py`, change the radar's y-noise from 12 to 2 (make it good at y
   too). Re-run. Watch the fused RMSE barely change — because the camera already
   covered y. That teaches you what "complementary" really means.
4. In `tracker.hpp`, bump `accel_noise` way up, re-run, and see the track get
   jittery (it now over-trusts noisy measurements). That teaches you what `Q` does.
5. Explain it out loud to a rubber duck (or me) using §8. If you can do that, you
   own it.

You built a real thing. Now you understand it. Go pass that interview. 🦾
