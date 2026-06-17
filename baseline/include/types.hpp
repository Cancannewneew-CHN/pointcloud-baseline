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
    float cluster_radius = 3.0f;   // mm, 欧氏聚类邻接半径
    int min_cluster_size = 100;    // keep_largest<=0 时: 小于此点数的簇视为游离噪声并删除
    int keep_largest = 1;          // >=1 时: 仅保留最大的该数目个簇(去除远处独立条带)
};

struct StepStats {
    std::size_t point_count = 0;
    double elapsed_ms = 0.0;
};
