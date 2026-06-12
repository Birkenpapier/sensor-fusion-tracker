#pragma once

#include <vector>

#include "sft/kalman_filter.hpp"
#include "sft/types.hpp"

namespace sft {

// Tuning for the constant-velocity tracker. The two sensors are deliberately
// *complementary*: the radar is precise along x and coarse along y, the EO
// sensor is the reverse. Fusing them recovers a track better than either axis
// of either sensor alone -- the whole point of the demo.
struct TrackerConfig {
    // Process noise: spectral density of the (unmodelled) acceleration,
    // m²/s⁴. Larger => the filter trusts measurements more / trajectory may
    // manoeuvre harder.
    double accel_noise = 4.0;

    // Per-sensor measurement standard deviations, metres.
    double radar_sigma_x = 1.5;
    double radar_sigma_y = 12.0;
    double eo_sigma_x    = 12.0;
    double eo_sigma_y    = 1.5;

    // Initial state covariance.
    double init_pos_var = 100.0;
    double init_vel_var = 100.0;
};

// Fuses a time-ordered stream of multi-sensor position fixes into a single
// smoothed [px, py, vx, vy] track using a constant-velocity motion model.
class Tracker {
public:
    explicit Tracker(const TrackerConfig& cfg = {});

    // Consume measurements (must be sorted by ascending time) and return one
    // fused estimate per measurement.
    std::vector<StateEstimate> run(const std::vector<Measurement>& measurements);

private:
    using KF = KalmanFilter<4, 2>;

    StateEstimate step(const Measurement& m);
    KF::StateMat transition(double dt) const;       // F
    KF::StateMat process_noise(double dt) const;    // Q
    KF::MeasMat  measurement_noise(SensorType s) const;  // R

    TrackerConfig cfg_;
    KF kf_;
    double last_t_ = 0.0;
    bool initialized_ = false;
};

}  // namespace sft
