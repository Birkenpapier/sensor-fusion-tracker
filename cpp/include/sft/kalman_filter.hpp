#pragma once

#include <Eigen/Dense>

namespace sft {

// A generic discrete-time linear Kalman filter.
//
// N = state dimension, M = measurement dimension. Sizes are compile-time so
// Eigen can stack-allocate every matrix and the compiler can fully inline the
// algebra (zero heap allocation on the hot path -- the property that matters
// for a real-time fusion node).
//
// The filter is model-agnostic: F, Q, H and R are supplied per call, so the
// same class fuses any number of sensors with different observation models.
template <int N, int M>
class KalmanFilter {
public:
    using StateVec = Eigen::Matrix<double, N, 1>;
    using StateMat = Eigen::Matrix<double, N, N>;
    using MeasVec  = Eigen::Matrix<double, M, 1>;
    using MeasMat  = Eigen::Matrix<double, M, M>;
    using ObsMat   = Eigen::Matrix<double, M, N>;
    using GainMat  = Eigen::Matrix<double, N, M>;

    void init(const StateVec& x0, const StateMat& P0) {
        x_ = x0;
        P_ = P0;
    }

    // Time update: project the state and its uncertainty forward.
    //   x = F x
    //   P = F P Fᵀ + Q
    void predict(const StateMat& F, const StateMat& Q) {
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    // Measurement update: fold a sensor reading z into the state.
    //   y = z - H x            (innovation)
    //   S = H P Hᵀ + R         (innovation covariance)
    //   K = P Hᵀ S⁻¹           (Kalman gain)
    //   x = x + K y
    //   P = (I-KH) P (I-KH)ᵀ + K R Kᵀ   (Joseph form)
    //
    // The Joseph-form covariance update is used instead of the shorter
    // (I-KH)P because it stays symmetric and positive-definite under floating
    // point round-off -- important when a filter runs for hours unattended.
    void update(const MeasVec& z, const ObsMat& H, const MeasMat& R) {
        const MeasVec  y = z - H * x_;
        const MeasMat  S = H * P_ * H.transpose() + R;
        const GainMat  K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;

        const StateMat I   = StateMat::Identity();
        const StateMat IKH = I - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    }

    const StateVec& state() const { return x_; }
    const StateMat& covariance() const { return P_; }

private:
    StateVec x_ = StateVec::Zero();
    StateMat P_ = StateMat::Identity();
};

}  // namespace sft
