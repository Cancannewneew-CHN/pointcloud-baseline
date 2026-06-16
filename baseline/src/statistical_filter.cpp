#include "statistical_filter.hpp"

#include "kdtree.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

namespace {

float compute_mean(const std::vector<float>& values) {
    if (values.empty()) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (float value : values) {
        sum += value;
    }
    return sum / static_cast<float>(values.size());
}

float compute_std(const std::vector<float>& values, float mean) {
    if (values.size() < 2) {
        return 0.0f;
    }

    float variance = 0.0f;
    for (float value : values) {
        const float diff = value - mean;
        variance += diff * diff;
    }
    variance /= static_cast<float>(values.size());
    return std::sqrt(variance);
}

}  // namespace

StepStats statistical_outlier_removal(const PointCloud& input,
                                      int nb_neighbors,
                                      float std_ratio,
                                      PointCloud& output) {
    const auto start = std::chrono::steady_clock::now();

    KdTree tree;
    tree.build(input);

    // kNN 查询彼此独立且 KD-tree 只读,按点区间切分多线程并行。
    std::vector<float> mean_distances(input.size(), 0.0f);
    const std::size_t n = input.size();
    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) {
        hw = 1;
    }
    const std::size_t num_threads =
        std::min<std::size_t>(hw, n == 0 ? 1 : n);

    auto worker = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            mean_distances[i] = tree.mean_knn_distance(input[i], nb_neighbors, i);
        }
    };

    if (num_threads <= 1) {
        worker(0, n);
    } else {
        std::vector<std::thread> pool;
        pool.reserve(num_threads);
        const std::size_t chunk = (n + num_threads - 1) / num_threads;
        for (std::size_t t = 0; t < num_threads; ++t) {
            const std::size_t begin = t * chunk;
            const std::size_t end = std::min(begin + chunk, n);
            if (begin >= end) {
                break;
            }
            pool.emplace_back(worker, begin, end);
        }
        for (auto& thread : pool) {
            thread.join();
        }
    }

    const float mu = compute_mean(mean_distances);
    const float sigma = compute_std(mean_distances, mu);
    const float threshold = mu + std_ratio * sigma;

    output.clear();
    output.reserve(input.size());

    for (std::size_t i = 0; i < input.size(); ++i) {
        if (mean_distances[i] <= threshold) {
            output.push_back(input[i]);
        }
    }

    const auto end = std::chrono::steady_clock::now();
    StepStats stats{};
    stats.point_count = output.size();
    stats.elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}
