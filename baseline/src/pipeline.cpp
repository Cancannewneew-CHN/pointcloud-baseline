#include "pipeline.hpp"

#include "diff_segmentation.hpp"
#include "ply_io.hpp"
#include "roi_crop.hpp"
#include "statistical_filter.hpp"
#include "voxel_downsample.hpp"

#include <cerrno>
#include <iostream>
#include <string>
#include <sys/stat.h>

namespace {

bool ensure_directory(const std::string& path) {
    if (path.empty()) {
        return false;
    }

    std::string current;
    current.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
        const char ch = path[i];
        current.push_back(ch);
        if (ch == '/' && current.size() > 1) {
            if (mkdir(current.c_str(), 0755) != 0) {
                // 已存在时继续
            }
        }
    }

    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool should_run(PipelineStep current, PipelineStep stop_after) {
    if (stop_after == PipelineStep::All) {
        return true;
    }
    switch (stop_after) {
        case PipelineStep::Roi:
            return current == PipelineStep::Roi;
        case PipelineStep::Voxel:
            return current == PipelineStep::Roi || current == PipelineStep::Voxel;
        case PipelineStep::Statistical:
            return current == PipelineStep::Roi || current == PipelineStep::Voxel ||
                   current == PipelineStep::Statistical;
        case PipelineStep::Diff:
            return current == PipelineStep::Roi || current == PipelineStep::Voxel ||
                   current == PipelineStep::Statistical || current == PipelineStep::Diff;
        default:
            return true;
    }
}

void print_step(const char* name, std::size_t input_count, const StepStats& stats) {
    std::cout << name << ": " << input_count << " -> " << stats.point_count
              << " points, " << stats.elapsed_ms << " ms\n";
}

}  // namespace

PipelineResult run_pipeline(const PointCloud& env,
                            const PointCloud& scene,
                            const PipelineConfig& config,
                            PipelineStep stop_after) {
    PipelineResult result{};

    if (should_run(PipelineStep::Roi, stop_after)) {
        result.env_roi_stats = crop_roi(env, config.roi, result.env_roi);
        result.scene_roi_stats = crop_roi(scene, config.roi, result.scene_roi);
    }

    if (stop_after == PipelineStep::Roi) {
        return result;
    }

    const PointCloud& env_after_roi = result.env_roi;
    const PointCloud& scene_after_roi = result.scene_roi;

    if (should_run(PipelineStep::Voxel, stop_after)) {
        result.env_voxel_stats =
            voxel_downsample_nearest_centroid(env_after_roi, config.voxel_size, result.env_voxel);
        result.scene_voxel_stats = voxel_downsample_nearest_centroid(
            scene_after_roi, config.voxel_size, result.scene_voxel);
    }

    if (stop_after == PipelineStep::Voxel) {
        return result;
    }

    const PointCloud& env_after_voxel = result.env_voxel;
    const PointCloud& scene_after_voxel = result.scene_voxel;

    if (should_run(PipelineStep::Statistical, stop_after)) {
        result.env_filter_stats = statistical_outlier_removal(
            env_after_voxel, config.nb_neighbors, config.std_ratio, result.env_filtered);
        result.scene_filter_stats = statistical_outlier_removal(
            scene_after_voxel, config.nb_neighbors, config.std_ratio, result.scene_filtered);
    }

    if (stop_after == PipelineStep::Statistical) {
        return result;
    }

    if (should_run(PipelineStep::Diff, stop_after)) {
        result.diff_stats = differential_segmentation(
            result.scene_filtered, result.env_filtered, config.diff_threshold, result.workpiece);
    }

    return result;
}

void print_stats(const PipelineResult& result,
                 PipelineStep stop_after,
                 std::size_t env_input_count,
                 std::size_t scene_input_count) {
    std::cout << "\n=== Pipeline Stats ===\n";

    if (stop_after == PipelineStep::Roi || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Voxel || stop_after == PipelineStep::Statistical ||
        stop_after == PipelineStep::Diff) {
        std::cout << "[ROI]\n";
        print_step("  env", env_input_count, result.env_roi_stats);
        print_step("  scene", scene_input_count, result.scene_roi_stats);
    }

    if (stop_after == PipelineStep::Voxel || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Statistical || stop_after == PipelineStep::Diff) {
        std::cout << "[Voxel Downsample]\n";
        print_step("  env", result.env_roi.size(), result.env_voxel_stats);
        print_step("  scene", result.scene_roi.size(), result.scene_voxel_stats);
    }

    if (stop_after == PipelineStep::Statistical || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Diff) {
        std::cout << "[Statistical Filter]\n";
        print_step("  env", result.env_voxel.size(), result.env_filter_stats);
        print_step("  scene", result.scene_voxel.size(), result.scene_filter_stats);
    }

    if (stop_after == PipelineStep::Diff || stop_after == PipelineStep::All) {
        std::cout << "[Differential Segmentation]\n";
        print_step("  workpiece", result.scene_filtered.size(), result.diff_stats);
    }
}

bool save_step_outputs(const std::string& output_dir,
                       const PipelineResult& result,
                       PipelineStep stop_after) {
    if (!ensure_directory(output_dir)) {
        return false;
    }

    if (stop_after == PipelineStep::Roi || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Voxel || stop_after == PipelineStep::Statistical ||
        stop_after == PipelineStep::Diff) {
        if (!save_ply(output_dir + "/env_roi.ply", result.env_roi)) {
            return false;
        }
        if (!save_ply(output_dir + "/scene_roi.ply", result.scene_roi)) {
            return false;
        }
    }

    if (stop_after == PipelineStep::Voxel || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Statistical || stop_after == PipelineStep::Diff) {
        if (!save_ply(output_dir + "/env_voxel.ply", result.env_voxel)) {
            return false;
        }
        if (!save_ply(output_dir + "/scene_voxel.ply", result.scene_voxel)) {
            return false;
        }
    }

    if (stop_after == PipelineStep::Statistical || stop_after == PipelineStep::All ||
        stop_after == PipelineStep::Diff) {
        if (!save_ply(output_dir + "/env_filtered.ply", result.env_filtered)) {
            return false;
        }
        if (!save_ply(output_dir + "/scene_filtered.ply", result.scene_filtered)) {
            return false;
        }
    }

    if (stop_after == PipelineStep::Diff || stop_after == PipelineStep::All) {
        if (!save_ply(output_dir + "/workpiece_clean.ply", result.workpiece)) {
            return false;
        }
    }

    return true;
}
