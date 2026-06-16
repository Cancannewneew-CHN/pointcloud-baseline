#pragma once

#include <cstddef>
#include <vector>

struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

using PointCloud = std::vector<Point3D>;

struct RoiBounds {
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
    float z_min = 0.0f;
    float z_max = 0.0f;
};

struct PipelineConfig {
    RoiBounds roi{};
    float voxel_size = 1.0f;       // mm，减小以保留稀疏工件细节
    int nb_neighbors = 15;
    float std_ratio = 4.0f;
    float diff_threshold = 3.0f;   // mm, 约 3x voxel_size
};

struct StepStats {
    std::size_t point_count = 0;
    double elapsed_ms = 0.0;
};
