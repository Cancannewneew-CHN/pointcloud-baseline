#pragma once

#include "types.hpp"

#include <string>

// 读取 PCD 点云文件 (PCL .pcd, v0.7)。
// 支持 DATA binary / ascii, 字段中需包含 x y z (float)。其余字段自动跳过。
// 非法点 (NaN/Inf) 会被自动剔除。
bool load_pcd(const std::string& path, PointCloud& cloud);
