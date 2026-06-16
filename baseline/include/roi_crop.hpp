#pragma once

#include "types.hpp"

StepStats crop_roi(const PointCloud& input, const RoiBounds& bounds, PointCloud& output);
