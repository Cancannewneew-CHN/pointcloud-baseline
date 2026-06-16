#pragma once

#include <cstddef>
#include <queue>
#include <vector>

#include "types.hpp"

class KdTree {
public:
    void build(const PointCloud& cloud);
    Point3D nearest_neighbor(const Point3D& query) const;

    // 查询 query 的 k 个最近邻平均距离。self_index 用于排除点自身
    // (传 static_cast<std::size_t>(-1) 表示不排除)。
    float mean_knn_distance(const Point3D& query, int k, std::size_t self_index) const;

    // 收集 query 半径 radius 内所有点的索引(含自身)。
    void radius_search(const Point3D& query, float radius, std::vector<int>& out_indices) const;

private:
    struct Node {
        Point3D point{};
        int axis = 0;
        int left = -1;
        int right = -1;
        int index = -1;
    };

    std::vector<Node> nodes_;
    int root_ = -1;

    int build_tree(std::vector<int>& indices, int begin, int end, int depth);
    void search_nearest(int node_index,
                        const Point3D& query,
                        float& best_dist_sq,
                        Point3D& best_point) const;
    void search_knn(int node_index,
                    const Point3D& query,
                    int k,
                    std::size_t self_index,
                    std::priority_queue<float>& heap) const;
    void search_radius(int node_index,
                       const Point3D& query,
                       float radius_sq,
                       std::vector<int>& out_indices) const;
};
