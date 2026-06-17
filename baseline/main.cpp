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
    if (step_name == "diff") {
        return PipelineStep::Diff;
    }
    if (step_name == "sor" || step_name == "statistical") {
        return PipelineStep::Sor;
    }
    if (step_name == "cluster") {
        return PipelineStep::Cluster;
    }
    if (step_name == "all") {
        return PipelineStep::All;
    }
    return PipelineStep::All;
}

const char* step_label(PipelineStep step) {
    switch (step) {
        case PipelineStep::Roi:
            return "ROI 裁剪";
        case PipelineStep::Voxel:
            return "ROI + 体素下采样";
        case PipelineStep::Diff:
            return "ROI + 体素 + 差分分割";
        case PipelineStep::Sor:
            return "ROI + 体素 + 差分 + 统计滤波";
        case PipelineStep::Cluster:
            return "ROI + 体素 + 差分 + 统计滤波 + 聚类去游离点";
        case PipelineStep::All:
        default:
            return "完整 Pipeline";
    }
}

void print_usage(const char* program) {
    std::cout
        << "用法:\n"
        << "  " << program << " [选项]\n\n"
        << "流程顺序: ROI -> 体素下采样 -> 差分分割 -> 统计滤波 -> 聚类去游离点\n\n"
        << "选项:\n"
        << "  --step <roi|voxel|diff|sor|cluster|all>  运行到哪一步 (默认 all)\n"
        << "  --env <path>            空环境点云 (默认 data/origin-only-environment.ply)\n"
        << "  --scene <path>          含工件点云 (默认 data/raw_subass_workpiece.ply)\n"
        << "  --output <dir>          输出目录 (默认 output/baseline)\n"
        << "  --voxel <mm>            体素边长\n"
        << "  --nb-neighbors <n>      统计滤波 kNN 邻居数\n"
        << "  --std-ratio <r>         统计滤波标准差倍数阈值\n"
        << "  --diff-threshold <mm>   差分最近邻距离阈值\n"
        << "  --cluster-radius <mm>   聚类邻接半径\n"
        << "  --min-cluster-size <n>  仅在 --keep-largest 0 时生效:删除小于该点数的簇\n"
        << "  --keep-largest <n>      仅保留最大的 n 个簇 (默认 1; 设 0 则用 min-cluster-size)\n"
        << "  --roi <xmin xmax ymin ymax zmin zmax>  ROI 包围盒 (mm)\n";
}

PipelineConfig default_config() {
    PipelineConfig config{};
    // 基于 origin-only-environment.ply 工作台层分析 (单位: mm)
    config.roi.x_min = -176.4f;
    config.roi.x_max = 1164.9f;
    config.roi.y_min = -111.2f;
    config.roi.y_max = 960.7f;
    config.roi.z_min = -150.0f;
    config.roi.z_max = 100.0f;
    config.voxel_size = 1.0f;
    config.nb_neighbors = 15;
    config.std_ratio = 4.0f;
    config.diff_threshold = 3.0f;
    config.cluster_radius = 3.0f;
    config.min_cluster_size = 100;
    config.keep_largest = 1;
    return config;
}

}  // namespace

int main(int argc, char** argv) {
    std::string env_path = "data/origin-only-environment.ply";
    std::string scene_path = "data/raw_subass_workpiece.ply";
    std::string output_dir = "output/baseline";
    PipelineStep step = PipelineStep::All;
    PipelineConfig config = default_config();

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (arg == "--step" && i + 1 < argc) {
            step = parse_step(argv[++i]);
        } else if (arg == "--env" && i + 1 < argc) {
            env_path = argv[++i];
        } else if (arg == "--scene" && i + 1 < argc) {
            scene_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--voxel" && i + 1 < argc) {
            config.voxel_size = std::stof(argv[++i]);
        } else if (arg == "--nb-neighbors" && i + 1 < argc) {
            config.nb_neighbors = std::stoi(argv[++i]);
        } else if (arg == "--std-ratio" && i + 1 < argc) {
            config.std_ratio = std::stof(argv[++i]);
        } else if (arg == "--diff-threshold" && i + 1 < argc) {
            config.diff_threshold = std::stof(argv[++i]);
        } else if (arg == "--cluster-radius" && i + 1 < argc) {
            config.cluster_radius = std::stof(argv[++i]);
        } else if (arg == "--min-cluster-size" && i + 1 < argc) {
            config.min_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--keep-largest" && i + 1 < argc) {
            config.keep_largest = std::stoi(argv[++i]);
        } else if (arg == "--roi" && i + 6 < argc) {
            config.roi.x_min = std::stof(argv[++i]);
            config.roi.x_max = std::stof(argv[++i]);
            config.roi.y_min = std::stof(argv[++i]);
            config.roi.y_max = std::stof(argv[++i]);
            config.roi.z_min = std::stof(argv[++i]);
            config.roi.z_max = std::stof(argv[++i]);
        } else {
            std::cerr << "未知或参数不全: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
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

    std::cout << "=== Baseline 点云预处理 ===\n";
    std::cout << "环境点云: " << env_path << " (" << env.size() << " points)\n";
    std::cout << "场景点云: " << scene_path << " (" << scene.size() << " points)\n";
    std::cout << "当前测试步骤: " << step_label(step) << "\n";
    std::cout << "ROI 范围 (mm): x[" << config.roi.x_min << ", " << config.roi.x_max << "], y["
              << config.roi.y_min << ", " << config.roi.y_max << "], z[" << config.roi.z_min
              << ", " << config.roi.z_max << "]\n";
    std::cout << "参数: voxel=" << config.voxel_size << " nb=" << config.nb_neighbors
              << " std_ratio=" << config.std_ratio << " diff=" << config.diff_threshold
              << " cluster_r=" << config.cluster_radius
              << " min_cluster=" << config.min_cluster_size
              << " keep_largest=" << config.keep_largest << "\n";

    const PipelineResult result = run_pipeline(env, scene, config, step);

    print_stats(result, step, env.size(), scene.size());

    if (!save_step_outputs(output_dir, result, step)) {
        std::cerr << "保存输出点云失败: " << output_dir << "\n";
        return 1;
    }

    std::cout << "\n输出目录: " << output_dir << "\n";

    return 0;
}
