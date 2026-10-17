#include "map_server/map_loader.hpp"

#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <msgpack.hpp>

namespace map_server {

namespace {

struct TerrainMapData {
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    std::vector<uint8_t> terrain;
};

TerrainMapData readTerrainMsgpack(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) throw std::runtime_error("failed to open terrain msgpack: " + path);

    std::vector<char> buffer(
        (std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    if (buffer.empty()) throw std::runtime_error("empty terrain msgpack: " + path);

    const auto handle = msgpack::unpack(buffer.data(), buffer.size());
    const auto object = handle.get();
    if (object.type != msgpack::type::MAP) {
        throw std::runtime_error("terrain msgpack root must be a map");
    }

    TerrainMapData data;
    for (uint32_t i = 0; i < object.via.map.size; ++i) {
        const auto& entry = object.via.map.ptr[i];
        std::string key;
        if (entry.key.type == msgpack::type::STR) {
            key.assign(entry.key.via.str.ptr, entry.key.via.str.size);
        } else if (entry.key.type == msgpack::type::BIN) {
            key.assign(entry.key.via.bin.ptr, entry.key.via.bin.size);
        } else {
            throw std::runtime_error("terrain msgpack key must be text or binary");
        }
        if (key == "width" || key == "height") {
            if (entry.val.type == msgpack::type::POSITIVE_INTEGER) {
                const int value = static_cast<int>(entry.val.via.u64);
                if (key == "width") data.width = value;
                else data.height = value;
            } else if (entry.val.type == msgpack::type::NEGATIVE_INTEGER) {
                const int value = static_cast<int>(entry.val.via.i64);
                if (key == "width") data.width = value;
                else data.height = value;
            } else {
                throw std::runtime_error(key + " field must be an integer");
            }
        }
        else if (key == "resolution") {
            if (entry.val.type == msgpack::type::FLOAT32) {
                data.resolution = entry.val.via.f64;
            } else if (entry.val.type == msgpack::type::FLOAT64) {
                data.resolution = entry.val.via.f64;
            } else {
                throw std::runtime_error("resolution field must be floating point");
            }
        } else if (key == "terrain") {
            if (entry.val.type != msgpack::type::BIN) {
                throw std::runtime_error("terrain field must be a binary blob");
            }
            const auto& bin = entry.val.via.bin;
            data.terrain.assign(
                reinterpret_cast<const uint8_t*>(bin.ptr),
                reinterpret_cast<const uint8_t*>(bin.ptr) + bin.size);
        }
    }

    if (data.width <= 0 || data.height <= 0 || data.resolution <= 0.0) {
        throw std::runtime_error("invalid terrain map dimensions or resolution");
    }
    const size_t expected = static_cast<size_t>(data.width) * data.height;
    if (data.terrain.size() != expected) {
        throw std::runtime_error("terrain data size mismatch");
    }
    return data;
}

} // namespace

nav_common::GridMap loadMapFile(
    const std::string& msgpack_path)
{
    const TerrainMapData terrain = readTerrainMsgpack(msgpack_path);
    const int w = terrain.width;
    const int h = terrain.height;

    nav_common::GridMap map;
    map.width = w;
    map.height = h;
    map.resolution = terrain.resolution;
    map.origin_x = 0.0;
    map.origin_y = 0.0;
    map.data.assign(static_cast<size_t>(w) * h, 0);

    // msgpack 的行序已经与地图坐标一致，row=0 对应世界 y 最小的一侧。
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const auto terrain_label = terrain.terrain[static_cast<size_t>(r) * w + c];
            map.data[static_cast<size_t>(r) * w + c] = terrain_label == 1 ? 100 : 0;
        }
    }
    return map;
}

} // namespace map_server
