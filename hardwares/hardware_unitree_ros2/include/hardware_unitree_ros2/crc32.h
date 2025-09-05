//
// CRC32 implementation for hardware_unitree_ros2
//

#ifndef HARDWARE_UNITREE_ROS2_CRC32_H
#define HARDWARE_UNITREE_ROS2_CRC32_H

#include <cstdint>

namespace hardware_unitree_ros2 {

/**
 * @brief Calculate CRC32 checksum for data buffer
 * @param ptr Pointer to data buffer (as uint32_t array)
 * @param len Length of data in uint32_t units
 * @return CRC32 checksum
 */
inline uint32_t crc32_core(const uint32_t *ptr, uint32_t len) {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else {
                CRC32 <<= 1;
            }

            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

} // namespace hardware_unitree_ros2

#endif // HARDWARE_UNITREE_ROS2_CRC32_H
