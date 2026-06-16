#include "voxel_downsample.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace {

struct VoxelKey {
    int ix = 0;
    int iy = 0;
    int iz = 0;

    bool operator==(const VoxelKey& other) const {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
        const std::uint64_t x = static_cast<std::uint32_t>(key.ix);
        const std::uint64_t y = static_cast<std::uint32_t>(key.iy);
        const std::uint64_t z = static_cast<std::uint32_t>(key.iz);
        return static_cast<std::size_t>((x * 73856093u) ^ (y * 19349663u) ^ (z * 83492791u));
    }
};

VoxelKey voxel_index(const Point3D& point, float voxel_size) {
    return {
        static_cast<int>(std::floor(point.x / voxel_size)),
        static_cast<int>(std::floor(point.y / voxel_size)),
        static_cast<int>(std::floor(point.z / voxel_size)),
    };
}

Point3D compute_centroid(const std::vector<const Point3D*>& points) {
    Point3D centroid{};
    if (points.empty()) {
        return centroid;
    }

    for (const Point3D* point : points) {
        centroid.x += point->x;
        centroid.y += point->y;
        centroid.z += point->z;
    }

    const float inv_count = 1.0f / static_cast<float>(points.size());
    centroid.x *= inv_count;
    centroid.y *= inv_count;
    centroid.z *= inv_count;
    return centroid;
}

float squared_distance(const Point3D& a, const Point3D& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

StepStats voxel_downsample_nearest_centroid(const PointCloud& input,
                                            float voxel_size,
                                            PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    std::unordered_map<VoxelKey, std::vector<const Point3D*>, VoxelKeyHash> voxel_map;
    voxel_map.reserve(input.size() / 4 + 1);

    for (const Point3D& point : input) {
        voxel_map[voxel_index(point, voxel_size)].push_back(&point);
    }

    output.clear();
    output.reserve(voxel_map.size());

    for (const auto& entry : voxel_map) {
        const auto& voxel_points = entry.second;
        const Point3D centroid = compute_centroid(voxel_points);

        const Point3D* nearest = voxel_points.front();
        float min_dist_sq = squared_distance(*nearest, centroid);

        for (const Point3D* point : voxel_points) {
            const float dist_sq = squared_distance(*point, centroid);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                nearest = point;
            }
        }

        output.push_back(*nearest);
    }

    const auto end = std::chrono::steady_clock::now();
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}
