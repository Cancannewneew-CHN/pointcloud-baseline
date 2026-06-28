#pragma once

#include "types.hpp"

// 去除工作台平面 (无环境点云时, 用几何方式代替差分背景剔除)。
// 思路: 工作台是一个已知高度附近的大平面, 把它及其下方的点删掉, 只保留台面之上的点。

enum class PlaneMethod {
    ZCut,    // 简单按高度切: 保留 z > z_cut 的点
    Ransac   // RANSAC 拟合台面平面, 保留"在平面上方且距平面 > thickness"的点(可容忍倾斜)
};

struct PlaneParams {
    PlaneMethod method = PlaneMethod::Ransac;
    // ZCut 用:
    float z_cut = -42.0f;        // 保留 z > z_cut
    // RANSAC 用:
    bool use_z_hint = false;     // 是否使用台面高度提示(false 则在全部点上自动拟合)
    float z_hint = -50.0f;       // 台面大致高度, 用于挑选拟合候选点
    float z_hint_band = 20.0f;   // 仅用 z 在 [z_hint-band, z_hint+band] 的点拟合台面
    float thickness = 5.0f;      // 平面厚度(mm): 距拟合平面 <= 该值视为台面
    int iterations = 200;        // RANSAC 迭代次数
    float min_horizontal_nz = 0.85f;  // 只接受法向量 nz >= 此值的近水平平面
};

// 删除台面, output 为台面之上的点。
StepStats remove_table_plane(const PointCloud& input, const PlaneParams& params,
                             PointCloud& output);
