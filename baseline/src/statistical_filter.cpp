#include "statistical_filter.hpp"

#include "kdtree.hpp"

#include <chrono>
#include <cmath>
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

    std::vector<float> mean_distances(input.size(), 0.0f);
    for (std::size_t i = 0; i < input.size(); ++i) {
        mean_distances[i] = tree.mean_knn_distance(input[i], nb_neighbors, i);
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
