#pragma once

#include "types.hpp"

// 欧氏聚类去游离点:以 radius 为邻接半径做区域生长连通。
//   keep_largest >= 1 时:仅保留点数最大的 keep_largest 个簇(适合工件为单一/少数
//                         连通体的场景,可去掉远处独立条带/噪声簇);
//   keep_largest <= 0 时:保留点数 >= min_cluster_size 的所有簇。
StepStats euclidean_cluster_filter(const PointCloud& input,
                                   float radius,
                                   int min_cluster_size,
                                   int keep_largest,
                                   PointCloud& output);
