#pragma once

#include <string>

#include "types.hpp"

// 流程顺序:ROI -> 体素下采样 -> 差分分割 -> 统计滤波 -> 聚类去游离点。
// 差分提到统计滤波之前,使统计滤波只作用于工件点(数量小,速度快),
// 同时统计滤波放在差分后,正好清理差分残留的背景噪声。
enum class PipelineStep {
    Roi,
    Voxel,
    Diff,
    Sor,
    Cluster,
    All
};

struct PipelineResult {
    PointCloud env_roi;
    PointCloud scene_roi;
    PointCloud env_voxel;
    PointCloud scene_voxel;
    PointCloud workpiece_diff;   // 差分分割后
    PointCloud workpiece_sor;    // 统计滤波后
    PointCloud workpiece_clean;  // 聚类去游离点后(最终结果)

    StepStats env_roi_stats{};
    StepStats scene_roi_stats{};
    StepStats env_voxel_stats{};
    StepStats scene_voxel_stats{};
    StepStats diff_stats{};
    StepStats sor_stats{};
    StepStats cluster_stats{};
};

PipelineResult run_pipeline(const PointCloud& env,
                            const PointCloud& scene,
                            const PipelineConfig& config,
                            PipelineStep stop_after);

void print_stats(const PipelineResult& result,
                 PipelineStep stop_after,
                 std::size_t env_input_count = 0,
                 std::size_t scene_input_count = 0);

bool save_step_outputs(const std::string& output_dir,
                       const PipelineResult& result,
                       PipelineStep stop_after);
