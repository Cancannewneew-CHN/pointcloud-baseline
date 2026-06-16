#include "cluster_filter.hpp"

#include "kdtree.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

StepStats euclidean_cluster_filter(const PointCloud& input,
                                   float radius,
                                   int min_cluster_size,
                                   PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    output.clear();

    if (input.empty() || min_cluster_size <= 1) {
        output = input;
        const auto end = std::chrono::steady_clock::now();
        StepStats stats{};
        stats.point_count = output.size();
        stats.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        return stats;
    }

    KdTree tree;
    tree.build(input);

    std::vector<std::uint8_t> visited(input.size(), 0);
    std::vector<int> queue;
    std::vector<int> cluster;
    std::vector<int> neighbors;

    output.reserve(input.size());

    for (std::size_t seed = 0; seed < input.size(); ++seed) {
        if (visited[seed]) {
            continue;
        }

        // 区域生长:从 seed 出发,BFS 收集半径连通的所有点。
        queue.clear();
        cluster.clear();
        queue.push_back(static_cast<int>(seed));
        visited[seed] = 1;

        std::size_t head = 0;
        while (head < queue.size()) {
            const int current = queue[head++];
            cluster.push_back(current);

            tree.radius_search(input[current], radius, neighbors);
            for (const int n : neighbors) {
                if (!visited[n]) {
                    visited[n] = 1;
                    queue.push_back(n);
                }
            }
        }

        if (static_cast<int>(cluster.size()) >= min_cluster_size) {
            for (const int idx : cluster) {
                output.push_back(input[idx]);
            }
        }
    }

    const auto end = std::chrono::steady_clock::now();
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}
