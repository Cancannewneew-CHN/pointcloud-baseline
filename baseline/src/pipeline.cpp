#include "pipeline.hpp"

#include "cluster_filter.hpp"
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
            mkdir(current.c_str(), 0755);
        }
    }

    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

int step_order(PipelineStep step) {
    switch (step) {
        case PipelineStep::Roi:
            return 0;
        case PipelineStep::Voxel:
            return 1;
        case PipelineStep::Diff:
            return 2;
        case PipelineStep::Sor:
            return 3;
        case PipelineStep::Cluster:
            return 4;
        case PipelineStep::All:
        default:
            return 100;
    }
}

// 是否需要(至少)运行到给定步骤。
bool reaches(PipelineStep stop_after, PipelineStep step) {
    return step_order(stop_after) >= step_order(step);
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

    // Step 1: ROI 裁剪(env / scene 用相同范围)
    result.env_roi_stats = crop_roi(env, config.roi, result.env_roi);
    result.scene_roi_stats = crop_roi(scene, config.roi, result.scene_roi);
    if (!reaches(stop_after, PipelineStep::Voxel)) {
        return result;
    }

    // Step 2: 体素下采样
    result.env_voxel_stats =
        voxel_downsample_nearest_centroid(result.env_roi, config.voxel_size, result.env_voxel);
    result.scene_voxel_stats =
        voxel_downsample_nearest_centroid(result.scene_roi, config.voxel_size, result.scene_voxel);
    if (!reaches(stop_after, PipelineStep::Diff)) {
        return result;
    }

    // Step 3: 差分分割(scene - env)。env 直接用体素结果建树,不做统计滤波,
    // 既省时又避免删掉 env 点导致 scene 噪声被误判为工件。
    result.diff_stats = differential_segmentation(
        result.scene_voxel, result.env_voxel, config.diff_threshold, result.workpiece_diff);
    if (!reaches(stop_after, PipelineStep::Sor)) {
        return result;
    }

    // Step 4: 统计离群点去除(只作用于工件点,数量小,速度快)
    result.sor_stats = statistical_outlier_removal(
        result.workpiece_diff, config.nb_neighbors, config.std_ratio, result.workpiece_sor);
    if (!reaches(stop_after, PipelineStep::Cluster)) {
        return result;
    }

    // Step 5: 欧氏聚类去游离点(删除零散小簇)
    result.cluster_stats = euclidean_cluster_filter(
        result.workpiece_sor, config.cluster_radius, config.min_cluster_size,
        result.workpiece_clean);

    return result;
}

void print_stats(const PipelineResult& result,
                 PipelineStep stop_after,
                 std::size_t env_input_count,
                 std::size_t scene_input_count) {
    std::cout << "\n=== Pipeline Stats ===\n";

    std::cout << "[ROI]\n";
    print_step("  env", env_input_count, result.env_roi_stats);
    print_step("  scene", scene_input_count, result.scene_roi_stats);

    if (reaches(stop_after, PipelineStep::Voxel)) {
        std::cout << "[Voxel Downsample]\n";
        print_step("  env", result.env_roi.size(), result.env_voxel_stats);
        print_step("  scene", result.scene_roi.size(), result.scene_voxel_stats);
    }

    if (reaches(stop_after, PipelineStep::Diff)) {
        std::cout << "[Differential Segmentation]\n";
        print_step("  workpiece", result.scene_voxel.size(), result.diff_stats);
    }

    if (reaches(stop_after, PipelineStep::Sor)) {
        std::cout << "[Statistical Filter]\n";
        print_step("  workpiece", result.workpiece_diff.size(), result.sor_stats);
    }

    if (reaches(stop_after, PipelineStep::Cluster)) {
        std::cout << "[Euclidean Cluster Filter]\n";
        print_step("  workpiece", result.workpiece_sor.size(), result.cluster_stats);
    }
}

bool save_step_outputs(const std::string& output_dir,
                       const PipelineResult& result,
                       PipelineStep stop_after) {
    if (!ensure_directory(output_dir)) {
        return false;
    }

    if (!save_ply(output_dir + "/env_roi.ply", result.env_roi)) {
        return false;
    }
    if (!save_ply(output_dir + "/scene_roi.ply", result.scene_roi)) {
        return false;
    }

    if (reaches(stop_after, PipelineStep::Voxel)) {
        if (!save_ply(output_dir + "/env_voxel.ply", result.env_voxel)) {
            return false;
        }
        if (!save_ply(output_dir + "/scene_voxel.ply", result.scene_voxel)) {
            return false;
        }
    }

    if (reaches(stop_after, PipelineStep::Diff)) {
        if (!save_ply(output_dir + "/workpiece_diff.ply", result.workpiece_diff)) {
            return false;
        }
    }

    if (reaches(stop_after, PipelineStep::Sor)) {
        if (!save_ply(output_dir + "/workpiece_sor.ply", result.workpiece_sor)) {
            return false;
        }
    }

    if (reaches(stop_after, PipelineStep::Cluster)) {
        if (!save_ply(output_dir + "/workpiece_clean.ply", result.workpiece_clean)) {
            return false;
        }
    }

    return true;
}
