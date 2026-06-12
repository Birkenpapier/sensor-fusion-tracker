#pragma once

#include <string>

namespace sft {

// Which physical sensor produced a measurement. Each sensor has its own
// noise characteristics (R matrix), set in the Tracker.
enum class SensorType { Radar, EO };

// A single position fix from one sensor at one instant.
// Both sensors report a Cartesian position here; the difference between them
// lives in their noise covariance, not in the measurement model.
struct Measurement {
    double t = 0.0;   // timestamp, seconds
    SensorType sensor = SensorType::Radar;
    double x = 0.0;   // measured position, metres
    double y = 0.0;
};

// One fused state estimate emitted by the tracker.
struct StateEstimate {
    double t = 0.0;
    double px = 0.0, py = 0.0;   // position, metres
    double vx = 0.0, vy = 0.0;   // velocity, m/s
    double pos_std = 0.0;        // sqrt of mean position variance (track quality)
};

inline SensorType parse_sensor(const std::string& s) {
    return (s == "eo" || s == "EO") ? SensorType::EO : SensorType::Radar;
}

inline const char* to_string(SensorType s) {
    return s == SensorType::EO ? "eo" : "radar";
}

}  // namespace sft
