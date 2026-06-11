#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace modbus_rtu {

constexpr uint8_t SLAVE_ID = 1;
constexpr uint16_t REG_TORQUE_ENABLE = 256;
constexpr uint16_t REG_GOAL_CURRENT = 275;
constexpr uint16_t REG_GOAL_POSITION = 282;
// RH-P12-RN-A Dynamixel control-table addresses divided by two for Modbus
// holding-register addressing: Present Current=574/2, Present Position=580/2.
constexpr uint16_t REG_PRESENT_CURRENT = 287;
constexpr uint16_t REG_PRESENT_POSITION = 290;

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

// FC03: Read Holding Registers - read present position (register 276, 2 registers)
inline std::vector<uint8_t> fc03_read_registers(uint16_t start, uint16_t count) {
    return make_frame({
        SLAVE_ID, 0x03,
        static_cast<uint8_t>((start >> 8) & 0xFF),
        static_cast<uint8_t>(start & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF),
        static_cast<uint8_t>(count & 0xFF)
    });
}

inline std::vector<uint8_t> fc03_read_present_state() {
    // Current(287), velocity(288-289), position(290-291)
    return fc03_read_registers(REG_PRESENT_CURRENT, 5);
}

inline bool valid_fc03_response(const std::vector<uint8_t>& response, size_t data_bytes) {
    if (response.size() < data_bytes + 5 || response[1] != 0x03) return false;
    uint8_t byte_count = response[2];
    return byte_count >= data_bytes;
}

inline int parse_present_current(const std::vector<uint8_t>& response) {
    if (!valid_fc03_response(response, 10)) return -1;
    const uint16_t raw = (static_cast<uint16_t>(response[3]) << 8) | response[4];
    return static_cast<int16_t>(raw);
}

inline int parse_present_position(const std::vector<uint8_t>& response) {
    if (!valid_fc03_response(response, 10)) return -1;
    // Modbus exposes the Dynamixel 32-bit value as low word then high word.
    const uint32_t low_word =
        (static_cast<uint32_t>(response[9]) << 8) | response[10];
    const uint32_t high_word =
        (static_cast<uint32_t>(response[11]) << 8) | response[12];
    const uint32_t position = low_word | (high_word << 16);
    if (position > 100000U) return -1;
    return static_cast<int>(position);
}

}  // namespace modbus_rtu
