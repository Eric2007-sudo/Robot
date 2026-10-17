#include "map_server/costmap_inflation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace map_server {

namespace {

constexpr double kInfinity = 1.0e20;

void distanceTransform1D(const std::vector<double>& input, std::vector<double>& output) {
    const int size = static_cast<int>(input.size());
    output.assign(static_cast<size_t>(size), kInfinity);

    std::vector<int> envelope(static_cast<size_t>(size));
    std::vector<double> boundaries(static_cast<size_t>(size) + 1);
    int envelope_size = 0;
    envelope[0] = 0;
    boundaries[0] = -kInfinity;
    boundaries[1] = kInfinity;

    for (int current = 1; current < size; ++current) {
        double boundary = 0.0;
        while (true) {
            const int previous = envelope[envelope_size];
            boundary = ((input[current] + static_cast<double>(current * current)) -
                        (input[previous] + static_cast<double>(previous * previous))) /
                       (2.0 * static_cast<double>(current - previous));
            if (envelope_size == 0 || boundary > boundaries[envelope_size]) break;
            --envelope_size;
        }
        ++envelope_size;
        envelope[envelope_size] = current;
        boundaries[envelope_size] = boundary;
        boundaries[envelope_size + 1] = kInfinity;
    }

    envelope_size = 0;
    for (int current = 0; current < size; ++current) {
        while (boundaries[envelope_size + 1] < static_cast<double>(current)) {
            ++envelope_size;
        }
        const double delta = static_cast<double>(current - envelope[envelope_size]);
        output[current] = delta * delta + input[envelope[envelope_size]];
    }
}

std::vector<double> distanceToObstacleSquared(const nav_common::GridMap& map) {
    const int width = map.width;
    const int height = map.height;
    std::vector<double> vertical(static_cast<size_t>(width) * height, kInfinity);
    std::vector<double> result(static_cast<size_t>(width) * height, kInfinity);

    for (int col = 0; col < width; ++col) {
        std::vector<double> input(static_cast<size_t>(height));
        std::vector<double> output;
        for (int row = 0; row < height; ++row) {
            input[row] = map.valueAt(row, col) >= 100 ? 0.0 : kInfinity;
        }
        distanceTransform1D(input, output);
        for (int row = 0; row < height; ++row) {
            vertical[static_cast<size_t>(row) * width + col] = output[row];
        }
    }

    for (int row = 0; row < height; ++row) {
        std::vector<double> input(static_cast<size_t>(width));
        std::vector<double> output;
        for (int col = 0; col < width; ++col) {
            input[col] = vertical[static_cast<size_t>(row) * width + col];
        }
        distanceTransform1D(input, output);
        for (int col = 0; col < width; ++col) {
            result[static_cast<size_t>(row) * width + col] = output[col];
        }
    }
    return result;
}

} // namespace

nav_common::GridMap inflateCostMap(
    const nav_common::GridMap& raw, const InflationParams& params)
{
    const int w = raw.width;
    const int h = raw.height;
    const auto distance_squared = distanceToObstacleSquared(raw);

    nav_common::GridMap cost = raw;
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const size_t id = static_cast<size_t>(r) * w + c;
            if (raw.valueAt(r, c) >= 100) {
                cost.data[id] = 100;  // lethal
                continue;
            }
            const double distance_m = std::sqrt(distance_squared[id]) * raw.resolution;
            if (distance_m <= params.full_cost_radius_m) {
                cost.data[id] = 99;
            } else if (distance_m <= params.cutoff_radius_m) {
                const double decayed = 99.0 * std::exp(
                    -params.decay_rate_per_m * (distance_m - params.full_cost_radius_m));
                cost.data[id] = static_cast<int8_t>(
                    std::clamp(std::lround(decayed), 1L, 98L));
            } else {
                cost.data[id] = 0;
            }
        }
    }
    return cost;
}

} // namespace map_server
