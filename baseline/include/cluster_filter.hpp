#pragma once

#include "types.hpp"

// 欧氏聚类去游离点:以 radius 为邻接半径做区域生长连通,
// 仅保留点数 >= min_cluster_size 的簇,删除零散小簇。
StepStats euclidean_cluster_filter(const PointCloud& input,
                                   float radius,
                                   int min_cluster_size,
                                   PointCloud& output);
