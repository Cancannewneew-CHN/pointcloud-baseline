#pragma once

#include "types.hpp"

StepStats voxel_downsample_nearest_centroid(const PointCloud& input,
                                            float voxel_size,
                                            PointCloud& output);
