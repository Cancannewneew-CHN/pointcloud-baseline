#pragma once

#include <string>

#include "types.hpp"

bool load_ply(const std::string& path, PointCloud& cloud);
bool save_ply(const std::string& path, const PointCloud& cloud);
