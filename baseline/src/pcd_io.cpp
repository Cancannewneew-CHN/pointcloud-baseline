#include "pcd_io.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct PcdField {
    std::string name;
    int size = 4;     // 字节数
    char type = 'F';  // F/I/U
    int count = 1;
    int offset = 0;   // 在每个点中的字节偏移
};

bool is_valid(float v) {
    return std::isfinite(v);
}

float read_field_as_float(const char* base, const PcdField& f) {
    if (f.type == 'F' && f.size == 4) {
        float v;
        std::memcpy(&v, base + f.offset, 4);
        return v;
    }
    if (f.type == 'F' && f.size == 8) {
        double v;
        std::memcpy(&v, base + f.offset, 8);
        return static_cast<float>(v);
    }
    if (f.type == 'I' && f.size == 4) {
        std::int32_t v;
        std::memcpy(&v, base + f.offset, 4);
        return static_cast<float>(v);
    }
    if (f.type == 'U' && f.size == 4) {
        std::uint32_t v;
        std::memcpy(&v, base + f.offset, 4);
        return static_cast<float>(v);
    }
    return 0.0f;
}

}  // namespace

bool load_pcd(const std::string& path, PointCloud& cloud) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    std::vector<std::string> fields_names;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    std::size_t point_count = 0;
    std::string data_type;  // "binary" / "ascii" / "binary_compressed"

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string key;
        iss >> key;
        if (key == "FIELDS" || key == "COLUMNS") {
            std::string tok;
            while (iss >> tok) fields_names.push_back(tok);
        } else if (key == "SIZE") {
            int v;
            while (iss >> v) sizes.push_back(v);
        } else if (key == "TYPE") {
            std::string v;
            while (iss >> v) types.push_back(v.empty() ? 'F' : v[0]);
        } else if (key == "COUNT") {
            int v;
            while (iss >> v) counts.push_back(v);
        } else if (key == "POINTS") {
            iss >> point_count;
        } else if (key == "WIDTH" && point_count == 0) {
            iss >> point_count;  // 兜底
        } else if (key == "DATA") {
            iss >> data_type;
            break;
        }
    }

    if (fields_names.empty() || point_count == 0) {
        return false;
    }
    if (counts.empty()) {
        counts.assign(fields_names.size(), 1);
    }

    std::vector<PcdField> fields;
    int offset = 0;
    int x_idx = -1, y_idx = -1, z_idx = -1;
    for (std::size_t i = 0; i < fields_names.size(); ++i) {
        PcdField f;
        f.name = fields_names[i];
        f.size = (i < sizes.size()) ? sizes[i] : 4;
        f.type = (i < types.size()) ? types[i] : 'F';
        f.count = (i < counts.size()) ? counts[i] : 1;
        f.offset = offset;
        offset += f.size * f.count;
        if (f.name == "x") x_idx = static_cast<int>(fields.size());
        if (f.name == "y") y_idx = static_cast<int>(fields.size());
        if (f.name == "z") z_idx = static_cast<int>(fields.size());
        fields.push_back(f);
    }
    const int stride = offset;
    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
        return false;  // 缺少坐标字段
    }

    cloud.clear();
    cloud.reserve(point_count);

    if (data_type == "ascii") {
        for (std::size_t i = 0; i < point_count; ++i) {
            if (!std::getline(file, line)) break;
            std::istringstream iss(line);
            std::vector<float> vals;
            float v;
            while (iss >> v) vals.push_back(v);
            if (vals.size() < fields_names.size()) continue;
            Point3D p{vals[x_idx], vals[y_idx], vals[z_idx]};
            if (is_valid(p.x) && is_valid(p.y) && is_valid(p.z)) {
                cloud.push_back(p);
            }
        }
        return !cloud.empty();
    }

    if (data_type != "binary") {
        return false;  // 暂不支持 binary_compressed
    }

    std::vector<char> buffer(static_cast<std::size_t>(stride) * point_count);
    file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize got = file.gcount();
    const std::size_t usable = static_cast<std::size_t>(got) / static_cast<std::size_t>(stride);

    for (std::size_t i = 0; i < usable; ++i) {
        const char* base = buffer.data() + i * static_cast<std::size_t>(stride);
        Point3D p;
        p.x = read_field_as_float(base, fields[x_idx]);
        p.y = read_field_as_float(base, fields[y_idx]);
        p.z = read_field_as_float(base, fields[z_idx]);
        if (is_valid(p.x) && is_valid(p.y) && is_valid(p.z)) {
            cloud.push_back(p);
        }
    }

    return !cloud.empty();
}
