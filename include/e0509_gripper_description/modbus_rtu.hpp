#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace modbus_rtu {

constexpr uint8_t SLAVE_ID = 1;
constexpr uint16_t REG_TORQUE_ENABLE = 256;
constexpr uint16_t REG_GOAL_POSITION = 282;

inline uint16_t crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (auto byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

inline std::vector<uint8_t> make_frame(const std::vector<uint8_t>& data) {
    auto frame = data;
    uint16_t crc = crc16(data);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    return frame;
}

inline std::vector<uint8_t> fc06_torque_enable() {
    return make_frame({SLAVE_ID, 0x06, 0x01, 0x00, 0x00, 0x01});
}

inline std::vector<uint8_t> fc16_position(int position) {
    position = std::clamp(position, 0, 700);
    return make_frame({
        SLAVE_ID, 0x10,
        0x01, 0x1A,
        0x00, 0x02,
        0x04,
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>(position & 0xFF),
        0x00, 0x00
    });
}

}  // namespace modbus_rtu
