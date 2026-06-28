// 无环境点云场景的预处理流程 (适用于 PCD 数据 + 已知工作台高度)。
// 流程: 读取 PCD -> [可选 ROI] -> 去台面(RANSAC/zcut) -> 体素下采样 -> 统计滤波 -> 聚类保留最大簇
//
// 与 env 差分版 (baseline) 的区别: 没有空环境点云, 改用"去工作台平面"做背景剔除。

#include "cluster_filter.hpp"
#include "pcd_io.hpp"
#include "plane_filter.hpp"
#include "ply_io.hpp"
#include "roi_crop.hpp"
#include "statistical_filter.hpp"
#include "types.hpp"
#include "voxel_downsample.hpp"

#include <cerrno>
#include <iostream>
#include <string>
#include <sys/stat.h>

namespace {

bool ensure_directory(const std::string& path) {
    std::string current;
    for (std::size_t i = 0; i < path.size(); ++i) {
        current.push_back(path[i]);
        if (path[i] == '/' && current.size() > 1) {
            mkdir(current.c_str(), 0755);
        }
    }
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool load_any(const std::string& path, PointCloud& cloud) {
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".pcd") {
        return load_pcd(path, cloud);
    }
    return load_ply(path, cloud);
}

void print_usage(const char* prog) {
    std::cout
        << "用法:\n  " << prog << " --input <pcd/ply> [选项]\n\n"
        << "无环境点云流程: 读取 -> [ROI] -> 去台面 -> 体素 -> 统计滤波 -> 聚类\n"
        << "去台面方法自动选择: 给了 --z-cut 走 zcut; 否则走 RANSAC 自动拟合。\n\n"
        << "选项:\n"
        << "  --input <path>          输入点云 (.pcd 或 .ply), 必填\n"
        << "  --output <dir>          输出目录 (默认 output/tabletop)\n"
        << "  --z-cut <mm>            给定则用 zcut: 保留 z > 该值\n"
        << "  --table-z <mm>          (可选) RANSAC 台面高度提示, 不给则全局自动拟合\n"
        << "  --plane-thickness <mm>  台面厚度容差, RANSAC 用 (默认 5)\n"
        << "  --voxel <mm>            体素边长 (默认 1.0)\n"
        << "  --nb-neighbors <n>      统计滤波 kNN 邻居数 (默认 15)\n"
        << "  --std-ratio <r>         统计滤波标准差倍数 (默认 3.0)\n"
        << "  --cluster-radius <mm>   聚类邻接半径 (默认 3.0)\n"
        << "  --keep-largest <n>      保留最大的 n 个簇 (默认 1; 0 用 min-cluster-size)\n"
        << "  --min-cluster-size <n>  keep-largest=0 时生效 (默认 100)\n"
        << "  --roi <xmin xmax ymin ymax zmin zmax>  可选 ROI 包围盒 (mm)\n";
}

void log_step(const char* name, std::size_t in, const StepStats& s) {
    std::cout << name << ": " << in << " -> " << s.point_count << " points, " << s.elapsed_ms
              << " ms\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string input_path;
    std::string output_dir = "output/tabletop";
    PlaneParams plane{};
    float voxel_size = 1.0f;
    int nb_neighbors = 15;
    float std_ratio = 3.0f;
    float cluster_radius = 3.0f;
    int keep_largest = 1;
    int min_cluster_size = 100;
    bool use_roi = false;
    RoiBounds roi{};
    bool z_cut_set = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--input" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--table-z" && i + 1 < argc) {
            plane.z_hint = std::stof(argv[++i]);
            plane.use_z_hint = true;
        } else if (arg == "--plane-thickness" && i + 1 < argc) {
            plane.thickness = std::stof(argv[++i]);
        } else if (arg == "--z-cut" && i + 1 < argc) {
            plane.z_cut = std::stof(argv[++i]);
            z_cut_set = true;
        } else if (arg == "--voxel" && i + 1 < argc) {
            voxel_size = std::stof(argv[++i]);
        } else if (arg == "--nb-neighbors" && i + 1 < argc) {
            nb_neighbors = std::stoi(argv[++i]);
        } else if (arg == "--std-ratio" && i + 1 < argc) {
            std_ratio = std::stof(argv[++i]);
        } else if (arg == "--cluster-radius" && i + 1 < argc) {
            cluster_radius = std::stof(argv[++i]);
        } else if (arg == "--keep-largest" && i + 1 < argc) {
            keep_largest = std::stoi(argv[++i]);
        } else if (arg == "--min-cluster-size" && i + 1 < argc) {
            min_cluster_size = std::stoi(argv[++i]);
        } else if (arg == "--roi" && i + 6 < argc) {
            use_roi = true;
            roi.x_min = std::stof(argv[++i]);
            roi.x_max = std::stof(argv[++i]);
            roi.y_min = std::stof(argv[++i]);
            roi.y_max = std::stof(argv[++i]);
            roi.z_min = std::stof(argv[++i]);
            roi.z_max = std::stof(argv[++i]);
        } else {
            std::cerr << "未知或参数不全: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (input_path.empty()) {
        std::cerr << "缺少 --input\n";
        print_usage(argv[0]);
        return 1;
    }

    // 自动选择去台面方法: 给了 --z-cut 走 zcut; 否则 RANSAC。
    plane.method = z_cut_set ? PlaneMethod::ZCut : PlaneMethod::Ransac;

    PointCloud cloud;
    if (!load_any(input_path, cloud)) {
        std::cerr << "无法读取点云: " << input_path << "\n";
        return 1;
    }

    std::cout << "=== Tabletop 预处理 (无环境点云) ===\n";
    std::cout << "输入: " << input_path << " (" << cloud.size() << " points)\n";
    if (plane.method == PlaneMethod::ZCut) {
        std::cout << "去台面方法: zcut (保留 z > " << plane.z_cut << ")\n";
    } else {
        std::cout << "去台面方法: ransac"
                  << (plane.use_z_hint ? "  台面高度提示 z_hint=" : "  无高度提示(全局自动拟合)");
        if (plane.use_z_hint) std::cout << plane.z_hint;
        std::cout << "  thickness=" << plane.thickness << "\n";
    }
    std::cout << "参数: voxel=" << voxel_size << " nb=" << nb_neighbors
              << " std_ratio=" << std_ratio << " cluster_r=" << cluster_radius
              << " keep_largest=" << keep_largest << "\n\n";

    if (!ensure_directory(output_dir)) {
        std::cerr << "无法创建输出目录: " << output_dir << "\n";
        return 1;
    }

    std::cout << "=== Stats ===\n";

    PointCloud work = cloud;

    // 可选 ROI
    if (use_roi) {
        PointCloud roi_out;
        const StepStats s = crop_roi(work, roi, roi_out);
        log_step("[ROI]        ", work.size(), s);
        work.swap(roi_out);
        save_ply(output_dir + "/01_roi.ply", work);
    }

    // 去台面
    {
        PointCloud plane_out;
        const StepStats s = remove_table_plane(work, plane, plane_out);
        log_step("[RemovePlane]", work.size(), s);
        work.swap(plane_out);
        save_ply(output_dir + "/02_no_table.ply", work);
    }

    // 体素下采样
    {
        PointCloud vox_out;
        const StepStats s = voxel_downsample_nearest_centroid(work, voxel_size, vox_out);
        log_step("[Voxel]      ", work.size(), s);
        work.swap(vox_out);
        save_ply(output_dir + "/03_voxel.ply", work);
    }

    // 统计滤波
    {
        PointCloud sor_out;
        const StepStats s = statistical_outlier_removal(work, nb_neighbors, std_ratio, sor_out);
        log_step("[SOR]        ", work.size(), s);
        work.swap(sor_out);
        save_ply(output_dir + "/04_sor.ply", work);
    }

    // 聚类保留最大簇
    {
        PointCloud clean;
        const StepStats s =
            euclidean_cluster_filter(work, cluster_radius, min_cluster_size, keep_largest, clean);
        log_step("[Cluster]    ", work.size(), s);
        work.swap(clean);
        save_ply(output_dir + "/05_workpiece_clean.ply", work);
    }

    std::cout << "\n最终工件点数: " << work.size() << "\n";
    std::cout << "输出目录: " << output_dir << "\n";
    return 0;
}
