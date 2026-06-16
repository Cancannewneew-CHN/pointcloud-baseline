#include "kdtree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace {

float squared_distance(const Point3D& a, const Point3D& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

float axis_value(const Point3D& point, int axis) {
    switch (axis) {
        case 0:
            return point.x;
        case 1:
            return point.y;
        default:
            return point.z;
    }
}

}  // namespace

void KdTree::build(const PointCloud& cloud) {
    nodes_.resize(cloud.size());
    std::vector<int> indices(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        nodes_[i].point = cloud[i];
        nodes_[i].index = static_cast<int>(i);
        indices[i] = static_cast<int>(i);
    }

    root_ = cloud.empty() ? -1 : build_tree(indices, 0, static_cast<int>(cloud.size()), 0);
}

int KdTree::build_tree(std::vector<int>& indices, int begin, int end, int depth) {
    if (begin >= end) {
        return -1;
    }

    const int axis = depth % 3;
    const int mid = (begin + end) / 2;

    std::nth_element(indices.begin() + begin,
                     indices.begin() + mid,
                     indices.begin() + end,
                     [axis, this](int lhs, int rhs) {
                         return axis_value(nodes_[lhs].point, axis) <
                                axis_value(nodes_[rhs].point, axis);
                     });

    const int node_index = indices[mid];
    nodes_[node_index].axis = axis;
    nodes_[node_index].left = build_tree(indices, begin, mid, depth + 1);
    nodes_[node_index].right = build_tree(indices, mid + 1, end, depth + 1);
    return node_index;
}

void KdTree::search_nearest(int node_index,
                            const Point3D& query,
                            float& best_dist_sq,
                            Point3D& best_point) const {
    if (node_index < 0) {
        return;
    }

    const Node& node = nodes_[node_index];
    const float dist_sq = squared_distance(query, node.point);
    if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_point = node.point;
    }

    const int axis = node.axis;
    const float query_axis = axis_value(query, axis);
    const float node_axis = axis_value(node.point, axis);
    const float diff = query_axis - node_axis;
    const float diff_sq = diff * diff;

    const int near_child = diff < 0.0f ? node.left : node.right;
    const int far_child = diff < 0.0f ? node.right : node.left;

    search_nearest(near_child, query, best_dist_sq, best_point);
    if (diff_sq < best_dist_sq) {
        search_nearest(far_child, query, best_dist_sq, best_point);
    }
}

Point3D KdTree::nearest_neighbor(const Point3D& query) const {
    float best_dist_sq = std::numeric_limits<float>::max();
    Point3D best_point{};
    search_nearest(root_, query, best_dist_sq, best_point);
    return best_point;
}

void KdTree::search_knn(int node_index,
                        const Point3D& query,
                        int k,
                        std::size_t self_index,
                        std::vector<float>& distances) const {
    if (node_index < 0 || k <= 0) {
        return;
    }

    const Node& node = nodes_[node_index];
    if (static_cast<std::size_t>(node.index) != self_index) {
        distances.push_back(std::sqrt(squared_distance(query, node.point)));
    }

    const int target_count = (self_index == static_cast<std::size_t>(-1)) ? k : k + 1;
    if (static_cast<int>(distances.size()) > target_count) {
        std::nth_element(distances.begin(),
                         distances.begin() + target_count,
                         distances.end());
        distances.resize(static_cast<std::size_t>(target_count));
    }

    float worst = distances.empty() ? std::numeric_limits<float>::max()
                                    : *std::max_element(distances.begin(), distances.end());

    const int axis = node.axis;
    const float query_axis = axis_value(query, axis);
    const float node_axis = axis_value(node.point, axis);
    const float diff = query_axis - node_axis;
    const float diff_sq = diff * diff;

    const int near_child = diff < 0.0f ? node.left : node.right;
    const int far_child = diff < 0.0f ? node.right : node.left;

    search_knn(near_child, query, k, self_index, distances);
    if (static_cast<int>(distances.size()) >= target_count) {
        worst = *std::max_element(distances.begin(), distances.end());
    }
    if (diff_sq < worst * worst || static_cast<int>(distances.size()) < k) {
        search_knn(far_child, query, k, self_index, distances);
    }
}

float KdTree::mean_knn_distance(const Point3D& query, int k, std::size_t self_index) const {
    if (k <= 0 || root_ < 0) {
        return 0.0f;
    }

    std::vector<float> distances;
    distances.reserve(static_cast<std::size_t>(k + 1));
    search_knn(root_, query, k, self_index, distances);

    if (static_cast<std::size_t>(self_index) != static_cast<std::size_t>(-1) &&
        static_cast<int>(distances.size()) > k) {
        std::nth_element(distances.begin(), distances.begin() + k, distances.end());
        distances.resize(static_cast<std::size_t>(k));
    } else if (static_cast<int>(distances.size()) > k) {
        std::nth_element(distances.begin(), distances.begin() + k, distances.end());
        distances.resize(static_cast<std::size_t>(k));
    }

    if (distances.empty()) {
        return 0.0f;
    }

    const float sum = std::accumulate(distances.begin(), distances.end(), 0.0f);
    return sum / static_cast<float>(distances.size());
}
