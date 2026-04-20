#pragma once

#include <cstdint>
#include <cstring>
#include <string>

#include "logger/logger.h"
#include "mc/can_driver.h"

namespace vid_transmit
{

constexpr uint32_t VID_CAN_ID = 0x180;
constexpr uint8_t VID_DLC = 8;
constexpr size_t VID_LENGTH = 7;

/**
 * @brief 计算 XOR 校验值
 * @param data 数据指针（至少 7 字节）
 * @return 校验值
 */
inline uint8_t calculate_xor_check(const uint8_t* data)
{
    uint8_t check = 0;
    for (size_t i = 0; i < VID_LENGTH; ++i)
    {
        check ^= data[i];
    }
    return check;
}

/**
 * @brief 验证 VID 字符串格式
 * @param vid VID 字符串
 * @return 是否有效
 */
inline bool validate_vid(const std::string& vid)
{
    if (vid.length() != VID_LENGTH)
    {
        LOG_WARN("VID length invalid: expected %zu, got %zu", VID_LENGTH, vid.length());
        return false;
    }

    for (char c : vid)
    {
        // 检查是否为 7 位 ASCII 可打印字符
        if (static_cast<uint8_t>(c) > 0x7F || c < 0x20)
        {
            LOG_WARN("VID contains invalid character: 0x%02X", static_cast<uint8_t>(c));
            return false;
        }
    }

    return true;
}

/**
 * @brief 构建 VID CAN 帧
 * @param vid VID 字符串（7个ASCII字符）
 * @param frame 输出的 CAN 帧
 * @return 是否成功
 */
inline bool build_vid_frame(const std::string& vid, CANFrame& frame)
{
    if (!validate_vid(vid))
    {
        return false;
    }

    frame.id = VID_CAN_ID;
    frame.dlc = VID_DLC;
    frame.ext = false;
    frame.rtr = false;

    // 复制 VID ASCII 字符
    std::memcpy(frame.data, vid.data(), VID_LENGTH);

    // 计算并填充校验值
    frame.data[7] = calculate_xor_check(frame.data);

    return true;
}

/**
 * @brief 发送 VID 到 CAN 总线
 * @param can_driver CAN 驱动对象引用
 * @param vid VID 字符串
 * @return 是否成功发送
 */
inline bool send_vid_to_can(CANDriver& can_driver, const std::string& vid)
{
    CANFrame frame;

    if (!build_vid_frame(vid, frame))
    {
        LOG_ERROR("Failed to build VID frame for VID: %s", vid.c_str());
        return false;
    }

    if (!can_driver.sendFrame(frame))
    {
        LOG_ERROR_THROTTLE(5000, "Failed to send VID frame to CAN bus");
        return false;
    }

    LOG_INFO_THROTTLE(5000, "VID sent to CAN bus: %s (CAN ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X)",
                      vid.c_str(), frame.id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4],
                      frame.data[5], frame.data[6], frame.data[7]);

    return true;
}

}  // namespace vid_transmit
