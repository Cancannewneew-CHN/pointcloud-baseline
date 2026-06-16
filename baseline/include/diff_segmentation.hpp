#pragma once

#include "types.hpp"

StepStats differential_segmentation(const PointCloud& scene,
                                    const PointCloud& env,
                                    float threshold,
                                    PointCloud& output);
