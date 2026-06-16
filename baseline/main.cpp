#include "pipeline.hpp"
#include "ply_io.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

PipelineStep parse_step(const std::string& step_name) {
    if (step_name == "roi") {
        return PipelineStep::Roi;
    }
    if (step_name == "voxel") {
        return PipelineStep::Voxel;
    }
    if (step_name == "statistical" || step_name == "sor") {
        return PipelineStep::Statistical;
    }
    if (step_name == "diff") {
        return PipelineStep::Diff;
    }
    if (step_name == "all") {
        return PipelineStep::All;
    }
    return PipelineStep::Roi;
}

void print_usage(const char* program) {
    std::cout
        << "用法:\n"
        << "  " << program
        << " [--step roi|voxel|statistical|diff|all] [--env path] [--scene path] [--output dir]\n\n"
        << "默认仅测试 ROI 裁剪 (step=roi)\n\n"
        << "示例:\n"
        << "  " << program << " --step roi\n"
        << "  " << program << " --step all --env data/env.ply --scene data/scene.ply\n";
}

PipelineConfig default_config() {
    PipelineConfig config{};
    // 基于 origin-only-environment.ply 工作台层分析 (单位: mm)
    // env 工作台 z[-120,-20] 的 5-95% 分位 + 边距
    config.roi.x_min = -176.4f;
    config.roi.x_max = 1164.9f;
    config.roi.y_min = -111.2f;
    config.roi.y_max = 960.7f;
    config.roi.z_min = -150.0f;
    config.roi.z_max = 100.0f;
    config.voxel_size = 1.0f;      // 2mm 会丢失约一半工件点，改为 1mm
    config.nb_neighbors = 15;
    config.std_ratio = 4.0f;       // 宽松统计滤波，避免误删稀疏工件
    config.diff_threshold = 3.0f;
    return config;
}

}  // namespace

int main(int argc, char** argv) {
    std::string env_path = "data/origin-only-environment.ply";
    std::string scene_path = "data/raw_subass_workpiece.ply";
    std::string output_dir = "output/baseline";
    PipelineStep step = PipelineStep::Roi;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "--step" && i + 1 < argc) {
            step = parse_step(argv[++i]);
            continue;
        }
        if (arg == "--env" && i + 1 < argc) {
            env_path = argv[++i];
            continue;
        }
        if (arg == "--scene" && i + 1 < argc) {
            scene_path = argv[++i];
            continue;
        }
        if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
            continue;
        }

        std::cerr << "未知参数: " << arg << "\n";
        print_usage(argv[0]);
        return 1;
    }

    PointCloud env;
    PointCloud scene;
    if (!load_ply(env_path, env)) {
        std::cerr << "无法读取环境点云: " << env_path << "\n";
        return 1;
    }
    if (!load_ply(scene_path, scene)) {
        std::cerr << "无法读取场景点云: " << scene_path << "\n";
        return 1;
    }

    const PipelineConfig config = default_config();

    std::cout << "=== Baseline 点云预处理 ===\n";
    std::cout << "环境点云: " << env_path << " (" << env.size() << " points)\n";
    std::cout << "场景点云: " << scene_path << " (" << scene.size() << " points)\n";
    std::cout << "当前测试步骤: ";
    switch (step) {
        case PipelineStep::Roi:
            std::cout << "ROI 裁剪\n";
            break;
        case PipelineStep::Voxel:
            std::cout << "ROI + 体素下采样\n";
            break;
        case PipelineStep::Statistical:
            std::cout << "ROI + 体素 + 统计滤波\n";
            break;
        case PipelineStep::Diff:
            std::cout << "ROI + 体素 + 统计滤波 + 差分分割\n";
            break;
        case PipelineStep::All:
            std::cout << "完整 Pipeline\n";
            break;
    }

    std::cout << "ROI 范围 (mm): x[" << config.roi.x_min << ", " << config.roi.x_max << "], y["
              << config.roi.y_min << ", " << config.roi.y_max << "], z[" << config.roi.z_min
              << ", " << config.roi.z_max << "]\n";

    const PipelineResult result = run_pipeline(env, scene, config, step);

    std::cout << "\n原始点数: env=" << env.size() << ", scene=" << scene.size() << "\n";
    print_stats(result, step, env.size(), scene.size());

    if (!save_step_outputs(output_dir, result, step)) {
        std::cerr << "保存输出点云失败: " << output_dir << "\n";
        return 1;
    }

    std::cout << "\n输出目录: " << output_dir << "\n";

    if (step == PipelineStep::Roi) {
        const std::size_t env_removed = env.size() - result.env_roi.size();
        const std::size_t scene_removed = scene.size() - result.scene_roi.size();
        std::cout << "\n[ROI 验证]\n";
        std::cout << "  env  裁剪掉 " << env_removed << " 点, 保留 " << result.env_roi.size()
                  << " 点\n";
        std::cout << "  scene裁剪掉 " << scene_removed << " 点, 保留 " << result.scene_roi.size()
                  << " 点\n";
        std::cout << "  输出文件: env_roi.ply, scene_roi.ply\n";
    }

    return 0;
}
