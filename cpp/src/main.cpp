// Sensor-fusion tracker -- command line entry point.
//
// Reads a time-sorted CSV of multi-sensor position fixes, runs the fusion
// tracker, and writes the fused track as CSV. CSV is used as the interface to
// the Python simulation/visualisation layer so each side stays in the language
// that suits it (see docs/adr/0002).
//
// Usage:  sft_app <measurements.csv> <out_track.csv>
//
//   measurements.csv : t,sensor,x,y          (sensor = "radar" | "eo")
//   out_track.csv    : t,px,py,vx,vy,pos_std

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "sft/tracker.hpp"

namespace {

std::vector<sft::Measurement> read_measurements(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open input file: " + path);
    }

    std::vector<sft::Measurement> out;
    std::string line;
    bool first = true;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (first) {  // skip header row
            first = false;
            if (line.rfind("t,", 0) == 0 || line.find("sensor") != std::string::npos) {
                continue;
            }
        }
        std::stringstream ss(line);
        std::string tok;
        sft::Measurement m;

        std::getline(ss, tok, ',');  m.t = std::stod(tok);
        std::getline(ss, tok, ',');  m.sensor = sft::parse_sensor(tok);
        std::getline(ss, tok, ',');  m.x = std::stod(tok);
        std::getline(ss, tok, ',');  m.y = std::stod(tok);
        out.push_back(m);
    }
    return out;
}

void write_track(const std::string& path, const std::vector<sft::StateEstimate>& track) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot open output file: " + path);
    }
    out << "t,px,py,vx,vy,pos_std\n";
    out.setf(std::ios::fixed);
    out.precision(6);
    for (const auto& e : track) {
        out << e.t << ',' << e.px << ',' << e.py << ',' << e.vx << ','
            << e.vy << ',' << e.pos_std << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: " << argv[0]
                  << " <measurements.csv> <out_track.csv>\n";
        return 1;
    }

    try {
        const auto measurements = read_measurements(argv[1]);
        if (measurements.empty()) {
            std::cerr << "no measurements read from " << argv[1] << '\n';
            return 1;
        }

        sft::Tracker tracker;
        const auto track = tracker.run(measurements);
        write_track(argv[2], track);

        std::cout << "fused " << measurements.size() << " measurements -> "
                  << track.size() << " estimates written to " << argv[2] << '\n';
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
