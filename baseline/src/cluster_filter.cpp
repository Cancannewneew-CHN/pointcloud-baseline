#include "cluster_filter.hpp"

#include "kdtree.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <vector>

StepStats euclidean_cluster_filter(const PointCloud& input,
                                   float radius,
                                   int min_cluster_size,
                                   int keep_largest,
                                   PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    output.clear();

    if (input.empty()) {
        const auto end = std::chrono::steady_clock::now();
        StepStats stats{};
        stats.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        return stats;
    }

    KdTree tree;
    tree.build(input);

    // 区域生长:为每个点分配簇标签。
    std::vector<int> label(input.size(), -1);
    std::vector<int> queue;
    std::vector<int> neighbors;
    std::vector<std::vector<int>> clusters;

    for (std::size_t seed = 0; seed < input.size(); ++seed) {
        if (label[seed] != -1) {
            continue;
        }

        const int cluster_id = static_cast<int>(clusters.size());
        clusters.emplace_back();
        std::vector<int>& members = clusters.back();

        queue.clear();
        queue.push_back(static_cast<int>(seed));
        label[seed] = cluster_id;

        std::size_t head = 0;
        while (head < queue.size()) {
            const int current = queue[head++];
            members.push_back(current);

            tree.radius_search(input[current], radius, neighbors);
            for (const int n : neighbors) {
                if (label[n] == -1) {
                    label[n] = cluster_id;
                    queue.push_back(n);
                }
            }
        }
    }

    // 决定保留哪些簇。
    std::vector<int> kept_ids;
    if (keep_largest >= 1) {
        std::vector<int> order(clusters.size());
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            order[i] = static_cast<int>(i);
        }
        std::sort(order.begin(), order.end(), [&clusters](int a, int b) {
            return clusters[a].size() > clusters[b].size();
        });
        const std::size_t n_keep =
            std::min<std::size_t>(static_cast<std::size_t>(keep_largest), order.size());
        for (std::size_t i = 0; i < n_keep; ++i) {
            kept_ids.push_back(order[i]);
        }
    } else {
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            if (static_cast<int>(clusters[i].size()) >= min_cluster_size) {
                kept_ids.push_back(static_cast<int>(i));
            }
        }
    }

    output.reserve(input.size());
    for (const int id : kept_ids) {
        for (const int idx : clusters[id]) {
            output.push_back(input[idx]);
        }
    }

    const auto end = std::chrono::steady_clock::now();
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}
