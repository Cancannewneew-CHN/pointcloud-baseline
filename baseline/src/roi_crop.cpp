#include "roi_crop.hpp"

#include <chrono>

bool point_in_roi(const Point3D& point, const RoiBounds& bounds) {
    return point.x >= bounds.x_min && point.x <= bounds.x_max &&
           point.y >= bounds.y_min && point.y <= bounds.y_max &&
           point.z >= bounds.z_min && point.z <= bounds.z_max;
}

StepStats crop_roi(const PointCloud& input, const RoiBounds& bounds, PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    output.clear();
    output.reserve(input.size());

    for (const auto& point : input) {
        if (point_in_roi(point, bounds)) {
            output.push_back(point);
        }
    }

    const auto end = std::chrono::steady_clock::now();
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}
