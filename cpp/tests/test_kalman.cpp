#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "sft/kalman_filter.hpp"
#include "sft/tracker.hpp"

using namespace sft;

namespace {

// A constant-velocity prediction must advance position by velocity*dt and
// leave velocity untouched.
TEST(KalmanFilter, ConstantVelocityPrediction) {
    KalmanFilter<4, 2> kf;
    Eigen::Vector4d x0(0.0, 0.0, 2.0, -1.0);  // moving (+2, -1) m/s
    kf.init(x0, Eigen::Matrix4d::Identity());

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    const double dt = 0.5;
    F(0, 2) = dt;
    F(1, 3) = dt;

    kf.predict(F, Eigen::Matrix4d::Zero());

    EXPECT_NEAR(kf.state()(0), 1.0, 1e-9);   // 0 + 2*0.5
    EXPECT_NEAR(kf.state()(1), -0.5, 1e-9);  // 0 + (-1)*0.5
    EXPECT_NEAR(kf.state()(2), 2.0, 1e-9);
    EXPECT_NEAR(kf.state()(3), -1.0, 1e-9);
}

// A measurement update must pull the estimate toward the observation and
// shrink the state uncertainty (covariance trace decreases).
TEST(KalmanFilter, UpdateReducesUncertaintyAndPullsToMeasurement) {
    KalmanFilter<4, 2> kf;
    Eigen::Vector4d x0(0.0, 0.0, 0.0, 0.0);
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 100.0;
    kf.init(x0, P0);

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 1.0;

    const double trace_before = kf.covariance().trace();
    Eigen::Vector2d z(10.0, 10.0);
    kf.update(z, H, R);

    // estimate moved toward the measurement...
    EXPECT_GT(kf.state()(0), 9.0);
    EXPECT_GT(kf.state()(1), 9.0);
    // ...and we are more certain than before.
    EXPECT_LT(kf.covariance().trace(), trace_before);
}

// The covariance must stay symmetric after an update (Joseph form guarantees
// this even under round-off).
TEST(KalmanFilter, CovarianceStaysSymmetric) {
    KalmanFilter<4, 2> kf;
    kf.init(Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity() * 50.0);

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    Eigen::Matrix2d R;
    R << 2.0, 0.3, 0.3, 5.0;

    kf.update(Eigen::Vector2d(3.0, 4.0), H, R);
    const Eigen::Matrix4d& P = kf.covariance();
    EXPECT_NEAR((P - P.transpose()).cwiseAbs().maxCoeff(), 0.0, 1e-9);
}

// End-to-end: fusing two complementary noisy sensors must track a straight-line
// target with smaller error than either sensor's noisy axis alone.
TEST(Tracker, FusionBeatsRawMeasurements) {
    // Ground-truth target moving diagonally at constant velocity.
    const double vx = 5.0, vy = 3.0;
    auto truth_at = [&](double t) {
        return std::pair<double, double>(vx * t, vy * t);
    };

    // Deterministic pseudo-noise (no RNG dependency in the unit test).
    auto noise = [](int k) { return std::sin(k * 1.7) * 0.6 + std::cos(k * 0.9) * 0.4; };

    TrackerConfig cfg;  // radar precise in x, EO precise in y
    std::vector<Measurement> meas;
    double sum_meas_err2 = 0.0;
    int n = 0;
    for (int k = 1; k <= 200; ++k) {
        const double t = k * 0.1;
        auto [tx, ty] = truth_at(t);

        Measurement radar;
        radar.t = t;
        radar.sensor = SensorType::Radar;
        radar.x = tx + cfg.radar_sigma_x * noise(k);
        radar.y = ty + cfg.radar_sigma_y * noise(k + 13);
        meas.push_back(radar);

        Measurement eo;
        eo.t = t;
        eo.sensor = SensorType::EO;
        eo.x = tx + cfg.eo_sigma_x * noise(k + 31);
        eo.y = ty + cfg.eo_sigma_y * noise(k + 47);
        meas.push_back(eo);

        for (const auto& m : {radar, eo}) {
            sum_meas_err2 += (m.x - tx) * (m.x - tx) + (m.y - ty) * (m.y - ty);
            ++n;
        }
    }
    const double rms_meas = std::sqrt(sum_meas_err2 / n);

    Tracker tracker(cfg);
    const auto track = tracker.run(meas);

    // Score fused error over the second half (after convergence).
    double sum_fused_err2 = 0.0;
    int m = 0;
    for (size_t i = track.size() / 2; i < track.size(); ++i) {
        auto [tx, ty] = truth_at(track[i].t);
        sum_fused_err2 += (track[i].px - tx) * (track[i].px - tx) +
                          (track[i].py - ty) * (track[i].py - ty);
        ++m;
    }
    const double rms_fused = std::sqrt(sum_fused_err2 / m);

    EXPECT_LT(rms_fused, rms_meas)
        << "fused RMS=" << rms_fused << " raw measurement RMS=" << rms_meas;
}

}  // namespace
