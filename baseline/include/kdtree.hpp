#pragma once

#include <cstddef>
#include <vector>

#include "types.hpp"

class KdTree {
public:
    void build(const PointCloud& cloud);
    Point3D nearest_neighbor(const Point3D& query) const;
    float mean_knn_distance(const Point3D& query, int k, std::size_t self_index) const;

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
                    std::vector<float>& distances) const;
};
