#include "kdtree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
    const float diff = axis_value(query, axis) - axis_value(node.point, axis);
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

// 用大小固定为 k 的最大堆维护当前最近的 k 个平方距离,堆顶即第 k 近距离,
// 用它做剪枝,避免每个节点都全量扫描距离向量。
void KdTree::search_knn(int node_index,
                        const Point3D& query,
                        int k,
                        std::size_t self_index,
                        std::priority_queue<float>& heap) const {
    if (node_index < 0) {
        return;
    }

    const Node& node = nodes_[node_index];
    if (static_cast<std::size_t>(node.index) != self_index) {
        const float dist_sq = squared_distance(query, node.point);
        if (static_cast<int>(heap.size()) < k) {
            heap.push(dist_sq);
        } else if (dist_sq < heap.top()) {
            heap.pop();
            heap.push(dist_sq);
        }
    }

    const int axis = node.axis;
    const float diff = axis_value(query, axis) - axis_value(node.point, axis);
    const float diff_sq = diff * diff;

    const int near_child = diff < 0.0f ? node.left : node.right;
    const int far_child = diff < 0.0f ? node.right : node.left;

    search_knn(near_child, query, k, self_index, heap);
    if (static_cast<int>(heap.size()) < k || diff_sq < heap.top()) {
        search_knn(far_child, query, k, self_index, heap);
    }
}

float KdTree::mean_knn_distance(const Point3D& query, int k, std::size_t self_index) const {
    if (k <= 0 || root_ < 0) {
        return 0.0f;
    }

    std::priority_queue<float> heap;  // 最大堆,存平方距离
    search_knn(root_, query, k, self_index, heap);

    if (heap.empty()) {
        return 0.0f;
    }

    const int count = static_cast<int>(heap.size());
    float sum = 0.0f;
    while (!heap.empty()) {
        sum += std::sqrt(heap.top());
        heap.pop();
    }
    return sum / static_cast<float>(count);
}

void KdTree::search_radius(int node_index,
                           const Point3D& query,
                           float radius_sq,
                           std::vector<int>& out_indices) const {
    if (node_index < 0) {
        return;
    }

    const Node& node = nodes_[node_index];
    if (squared_distance(query, node.point) <= radius_sq) {
        out_indices.push_back(node.index);
    }

    const int axis = node.axis;
    const float diff = axis_value(query, axis) - axis_value(node.point, axis);

    const int near_child = diff < 0.0f ? node.left : node.right;
    const int far_child = diff < 0.0f ? node.right : node.left;

    search_radius(near_child, query, radius_sq, out_indices);
    if (diff * diff <= radius_sq) {
        search_radius(far_child, query, radius_sq, out_indices);
    }
}

void KdTree::radius_search(const Point3D& query, float radius, std::vector<int>& out_indices) const {
    out_indices.clear();
    if (root_ < 0) {
        return;
    }
    search_radius(root_, query, radius * radius, out_indices);
}
