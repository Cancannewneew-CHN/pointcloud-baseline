#include "plane_filter.hpp"

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

namespace {

struct Plane {
    // n.x*x + n.y*y + n.z*z + d = 0, n 为单位法向量(朝上, n.z > 0)
    float nx = 0.0f, ny = 0.0f, nz = 1.0f, d = 0.0f;
};

// 用三点拟合平面。返回 false 表示三点共线。
bool plane_from_3(const Point3D& a, const Point3D& b, const Point3D& c, Plane& plane) {
    const float ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    const float vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
    float nx = uy * vz - uz * vy;
    float ny = uz * vx - ux * vz;
    float nz = ux * vy - uy * vx;
    const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-6f) {
        return false;
    }
    nx /= len;
    ny /= len;
    nz /= len;
    // 让法向量朝上(+z), 便于用符号距离判断"上方"
    if (nz < 0.0f) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
    }
    plane.nx = nx;
    plane.ny = ny;
    plane.nz = nz;
    plane.d = -(nx * a.x + ny * a.y + nz * a.z);
    return true;
}

inline float signed_distance(const Plane& p, const Point3D& q) {
    return p.nx * q.x + p.ny * q.y + p.nz * q.z + p.d;
}

StepStats finish(const PointCloud& output, std::chrono::steady_clock::time_point start) {
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return stats;
}

}  // namespace

StepStats remove_table_plane(const PointCloud& input, const PlaneParams& params,
                             PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();
    output.clear();

    if (params.method == PlaneMethod::ZCut) {
        output.reserve(input.size() / 4 + 1);
        for (const auto& p : input) {
            if (p.z > params.z_cut) {
                output.push_back(p);
            }
        }
        return finish(output, start);
    }

    // ---- RANSAC ----
    // 1) 选出用于拟合台面的候选点。
    //    use_z_hint=true : 仅取 z 在 z_hint 附近的点(更快, 更稳, 防止抽到工件面);
    //    use_z_hint=false: 用全部点自动拟合(配合下方"近水平"约束找最大水平面)。
    std::vector<int> candidates;
    if (params.use_z_hint) {
        candidates.reserve(input.size());
        const float lo = params.z_hint - params.z_hint_band;
        const float hi = params.z_hint + params.z_hint_band;
        for (std::size_t i = 0; i < input.size(); ++i) {
            if (input[i].z >= lo && input[i].z <= hi) {
                candidates.push_back(static_cast<int>(i));
            }
        }
    } else {
        // 全部点参与; 评分集过大时按步长抽样以控制耗时(删除仍作用于全部点)。
        const std::size_t kMaxScore = 400000;
        const std::size_t stride =
            input.size() > kMaxScore ? input.size() / kMaxScore : 1;
        candidates.reserve(input.size() / stride + 1);
        for (std::size_t i = 0; i < input.size(); i += stride) {
            candidates.push_back(static_cast<int>(i));
        }
    }
    if (candidates.size() < 3) {
        // 候选不足, 退化为按高度切
        const float fallback_z = params.use_z_hint ? params.z_hint : params.z_cut;
        output.reserve(input.size());
        for (const auto& p : input) {
            if (p.z > fallback_z + params.thickness) {
                output.push_back(p);
            }
        }
        return finish(output, start);
    }

    // 2) RANSAC 找内点最多的平面。
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
    Plane best{};
    std::size_t best_inliers = 0;
    for (int it = 0; it < params.iterations; ++it) {
        const Point3D& a = input[candidates[pick(rng)]];
        const Point3D& b = input[candidates[pick(rng)]];
        const Point3D& c = input[candidates[pick(rng)]];
        Plane plane;
        if (!plane_from_3(a, b, c, plane)) {
            continue;
        }
        // 只接受接近水平的平面(法向量与 +z 夹角小), 避免拟合到工件侧壁。
        if (plane.nz < params.min_horizontal_nz) {
            continue;
        }
        std::size_t inliers = 0;
        for (const int idx : candidates) {
            if (std::fabs(signed_distance(plane, input[idx])) <= params.thickness) {
                ++inliers;
            }
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best = plane;
        }
    }

    if (best_inliers == 0) {
        const float fallback_z = params.use_z_hint ? params.z_hint : params.z_cut;
        output.reserve(input.size());
        for (const auto& p : input) {
            if (p.z > fallback_z + params.thickness) {
                output.push_back(p);
            }
        }
        return finish(output, start);
    }

    // 3) 保留"平面上方且距平面 > thickness"的点(同时去掉台面本身与台面下方噪声)。
    output.reserve(input.size() / 4 + 1);
    for (const auto& p : input) {
        if (signed_distance(best, p) > params.thickness) {
            output.push_back(p);
        }
    }
    return finish(output, start);
}
