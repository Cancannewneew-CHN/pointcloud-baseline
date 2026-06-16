#include "diff_segmentation.hpp"

#include "kdtree.hpp"

#include <chrono>
#include <cmath>

namespace {

float squared_distance(const Point3D& a, const Point3D& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

StepStats differential_segmentation(const PointCloud& scene,
                                    const PointCloud& env,
                                    float threshold,
                                    PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    output.clear();
    if (scene.empty()) {
        StepStats stats{};
        return stats;
    }

    if (env.empty()) {
        output = scene;
        const auto end = std::chrono::steady_clock::now();
        StepStats stats{};
        stats.point_count = output.size();
        stats.elapsed_ms =
            std::chrono::duration<double, std::milli>(end - start).count();
        return stats;
    }

    KdTree env_tree;
    env_tree.build(env);
    const float threshold_sq = threshold * threshold;

    output.reserve(scene.size());
    for (const Point3D& point : scene) {
        const Point3D nearest = env_tree.nearest_neighbor(point);
        if (squared_distance(point, nearest) > threshold_sq) {
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
