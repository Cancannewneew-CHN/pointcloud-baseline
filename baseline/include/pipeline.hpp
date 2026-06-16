#pragma once

#include <string>

#include "types.hpp"

enum class PipelineStep {
    Roi,
    Voxel,
    Statistical,
    Diff,
    All
};

struct PipelineResult {
    PointCloud env_roi;
    PointCloud scene_roi;
    PointCloud env_voxel;
    PointCloud scene_voxel;
    PointCloud env_filtered;
    PointCloud scene_filtered;
    PointCloud workpiece;

    StepStats env_roi_stats{};
    StepStats scene_roi_stats{};
    StepStats env_voxel_stats{};
    StepStats scene_voxel_stats{};
    StepStats env_filter_stats{};
    StepStats scene_filter_stats{};
    StepStats diff_stats{};
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
