#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

class ROMParser {
public:
    explicit ROMParser(const std::string& filename) {
        std::ifstream rom_file(filename, std::ios::binary);
        if (!rom_file) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        std::vector<char> rom_data((std::istreambuf_iterator<char>(rom_file)), std::istreambuf_iterator<char>());
        parse_rom_data(rom_data);
    }

    const std::vector<float>& GetMarkerPositions() const {
        return marker_positions;
    }

    const int& GetNumMarkers() const {
        return num_markers;
    }

private:
    std::vector<float> marker_positions;
    int num_markers = 4;

    void parse_rom_data(const std::vector<char>& rom_data) {
        num_markers = static_cast<unsigned char>(rom_data[28]);
        int pos = 72;
        for (int i = 0; i < num_markers; ++i) {
            float x, y, z;
            std::memcpy(&x, &rom_data[pos], sizeof(float));
            std::memcpy(&y, &rom_data[pos + 4], sizeof(float));
            std::memcpy(&z, &rom_data[pos + 8], sizeof(float));

            x = roundf(x * 1000) / 1000.0f;
            y = roundf(y * 1000) / 1000.0f;
            z = roundf(z * 1000) / 1000.0f;

            x = (x == -0.0f) ? 0.0f : x;
            y = (y == -0.0f) ? 0.0f : y;
            z = (z == -0.0f) ? 0.0f : z;

            marker_positions.push_back(x);
            marker_positions.push_back(y);
            marker_positions.push_back(z);

            pos += 12;
        }
    }
};