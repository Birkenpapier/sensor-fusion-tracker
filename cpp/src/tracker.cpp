#include "sft/tracker.hpp"

#include <cmath>

namespace sft {

Tracker::Tracker(const TrackerConfig& cfg) : cfg_(cfg) {}

// Constant-velocity state transition. State is [px, py, vx, vy].
//   px' = px + vx·dt
//   py' = py + vy·dt
Tracker::KF::StateMat Tracker::transition(double dt) const {
    KF::StateMat F = KF::StateMat::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
}

// Discrete white-noise-acceleration process model. Per axis the noise couples
// position and velocity through the well-known [[dt⁴/4, dt³/2],[dt³/2, dt²]]
// block, scaled by the acceleration spectral density.
Tracker::KF::StateMat Tracker::process_noise(double dt) const {
    const double q  = cfg_.accel_noise;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;

    KF::StateMat Q = KF::StateMat::Zero();
    // x axis: indices 0 (pos), 2 (vel)
    Q(0, 0) = dt4 / 4.0 * q;  Q(0, 2) = dt3 / 2.0 * q;
    Q(2, 0) = dt3 / 2.0 * q;  Q(2, 2) = dt2 * q;
    // y axis: indices 1 (pos), 3 (vel)
    Q(1, 1) = dt4 / 4.0 * q;  Q(1, 3) = dt3 / 2.0 * q;
    Q(3, 1) = dt3 / 2.0 * q;  Q(3, 3) = dt2 * q;
    return Q;
}

Tracker::KF::MeasMat Tracker::measurement_noise(SensorType s) const {
    const double sx = (s == SensorType::Radar) ? cfg_.radar_sigma_x : cfg_.eo_sigma_x;
    const double sy = (s == SensorType::Radar) ? cfg_.radar_sigma_y : cfg_.eo_sigma_y;
    KF::MeasMat R = KF::MeasMat::Zero();
    R(0, 0) = sx * sx;
    R(1, 1) = sy * sy;
    return R;
}

StateEstimate Tracker::step(const Measurement& m) {
    // Both sensors observe position directly.
    KF::ObsMat H = KF::ObsMat::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    if (!initialized_) {
        // Seed the state at the first fix with zero velocity and a wide prior.
        KF::StateVec x0;
        x0 << m.x, m.y, 0.0, 0.0;
        KF::StateMat P0 = KF::StateMat::Zero();
        P0(0, 0) = P0(1, 1) = cfg_.init_pos_var;
        P0(2, 2) = P0(3, 3) = cfg_.init_vel_var;
        kf_.init(x0, P0);
        last_t_ = m.t;
        initialized_ = true;
    } else {
        const double dt = m.t - last_t_;
        if (dt > 0.0) {
            kf_.predict(transition(dt), process_noise(dt));
            last_t_ = m.t;
        }
        KF::MeasVec z;
        z << m.x, m.y;
        kf_.update(z, H, measurement_noise(m.sensor));
    }

    const auto& x = kf_.state();
    const auto& P = kf_.covariance();
    StateEstimate est;
    est.t = m.t;
    est.px = x(0);  est.py = x(1);
    est.vx = x(2);  est.vy = x(3);
    est.pos_std = std::sqrt(0.5 * (P(0, 0) + P(1, 1)));
    return est;
}

std::vector<StateEstimate> Tracker::run(const std::vector<Measurement>& measurements) {
    std::vector<StateEstimate> out;
    out.reserve(measurements.size());
    for (const auto& m : measurements) {
        out.push_back(step(m));
    }
    return out;
}

}  // namespace sft
