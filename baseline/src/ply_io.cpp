#include "ply_io.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class PlyFormat {
    Ascii,
    BinaryLittleEndian
};

struct PlyHeaderInfo {
    PlyFormat format = PlyFormat::Ascii;
    std::size_t vertex_count = 0;
};

bool parse_vertex_count(const std::string& line, std::size_t& count) {
    const std::string prefix = "element vertex ";
    if (line.rfind(prefix, 0) != 0) {
        return false;
    }
    count = static_cast<std::size_t>(std::stoul(line.substr(prefix.size())));
    return true;
}

bool parse_header(std::istream& input, PlyHeaderInfo& header) {
    std::string line;
    if (!std::getline(input, line) || line != "ply") {
        return false;
    }

    bool header_done = false;
    while (std::getline(input, line)) {
        if (line == "format ascii 1.0") {
            header.format = PlyFormat::Ascii;
            continue;
        }
        if (line == "format binary_little_endian 1.0") {
            header.format = PlyFormat::BinaryLittleEndian;
            continue;
        }
        if (parse_vertex_count(line, header.vertex_count)) {
            continue;
        }
        if (line == "end_header") {
            header_done = true;
            break;
        }
    }

    return header_done && header.vertex_count > 0;
}

bool load_ascii_vertices(std::istream& input, std::size_t vertex_count, PointCloud& cloud) {
    cloud.clear();
    cloud.reserve(vertex_count);

    for (std::size_t i = 0; i < vertex_count; ++i) {
        std::string line;
        if (!std::getline(input, line)) {
            cloud.clear();
            return false;
        }

        std::istringstream iss(line);
        Point3D point{};
        if (!(iss >> point.x >> point.y >> point.z)) {
            cloud.clear();
            return false;
        }
        cloud.push_back(point);
    }

    return true;
}

bool load_binary_vertices(std::istream& input, std::size_t vertex_count, PointCloud& cloud) {
    cloud.resize(vertex_count);
    input.read(reinterpret_cast<char*>(cloud.data()),
               static_cast<std::streamsize>(vertex_count * sizeof(Point3D)));
    return static_cast<bool>(input);
}

void write_ply_header(std::ostream& output, std::size_t vertex_count, PlyFormat format) {
    output << "ply\n";
    if (format == PlyFormat::Ascii) {
        output << "format ascii 1.0\n";
    } else {
        output << "format binary_little_endian 1.0\n";
        output << "comment baseline generated\n";
    }
    output << "element vertex " << vertex_count << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "end_header\n";
}

}  // namespace

bool load_ply(const std::string& path, PointCloud& cloud) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    PlyHeaderInfo header{};
    if (!parse_header(file, header)) {
        return false;
    }

    if (header.format == PlyFormat::Ascii) {
        return load_ascii_vertices(file, header.vertex_count, cloud);
    }
    return load_binary_vertices(file, header.vertex_count, cloud);
}

bool save_ply(const std::string& path, const PointCloud& cloud) {
    const bool use_binary = cloud.size() > 500000;
    const PlyFormat format = use_binary ? PlyFormat::BinaryLittleEndian : PlyFormat::Ascii;

    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    write_ply_header(file, cloud.size(), format);

    if (format == PlyFormat::Ascii) {
        for (const auto& point : cloud) {
            file << point.x << " " << point.y << " " << point.z << "\n";
        }
    } else {
        file.write(reinterpret_cast<const char*>(cloud.data()),
                   static_cast<std::streamsize>(cloud.size() * sizeof(Point3D)));
    }

    return static_cast<bool>(file);
}
