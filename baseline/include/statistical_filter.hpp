#pragma once

#include "types.hpp"

StepStats statistical_outlier_removal(const PointCloud& input,
                                      int nb_neighbors,
                                      float std_ratio,
                                      PointCloud& output);
