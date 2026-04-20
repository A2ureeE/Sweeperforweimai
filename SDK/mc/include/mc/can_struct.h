#ifndef CAN_STRUCT_H
#define CAN_STRUCT_H

#include <string.h>

#include <array>
#include <cstdint>
#include <iostream>

#include "can_driver.h"

#pragma pack(push, 1)  // 禁止结构体补齐，确保8字节紧密排列

struct can_BMS_query_0x100
{
    // 报文ID与控制标志定义
    static constexpr uint32_t CMD_ID = 0x100;  // 标准帧ID
    static constexpr bool EXT_FLAG = false;    // 11位标准帧
    static constexpr bool RTR_FLAG = true;     // 普通数据帧（非远程帧）
    static constexpr uint8_t DLC = 8;          // 数据长度为8字节

    // 数据区（主机查询时通常发送全0）
    uint8_t data[8];

    // 构造函数：初始化数据为0
    can_BMS_query_0x100() { memset(data, 0, sizeof(data)); }

    // 生成CAN帧
    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        frame.dlc = DLC;
        memcpy(frame.data, data, sizeof(data));
        return frame;
    }
};

struct can_BMS_query_0x101
{
    // 报文ID与控制标志定义
    static constexpr uint32_t CMD_ID = 0x101;  // 标准帧ID
    static constexpr bool EXT_FLAG = false;    // 11位标准帧
    static constexpr bool RTR_FLAG = true;     // 普通数据帧（非远程帧）
    static constexpr uint8_t DLC = 8;          // 数据长度为8字节

    // 数据区（主机查询时通常发送全0）
    uint8_t data[8];

    // 构造函数：初始化数据为0
    can_BMS_query_0x101() { memset(data, 0, sizeof(data)); }

    // 生成CAN帧
    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        frame.dlc = DLC;
        memcpy(frame.data, data, sizeof(data));
        return frame;
    }
};

union can_MCU_cmd_union
{
    struct
    {
        // Byte 0: 设定转速低字节
        uint8_t speed_low;

        // Byte 1: 设定转速高字节
        uint8_t speed_high;

        // Byte 2: CAN使能信号 (1使能，其他值不使能)
        uint8_t can_enabled;

        // Byte 3: 开关信号状态
        struct
        {
            uint8_t cmd_status : 2;       // BIT0-1: 命令状态(0空挡,1后退,2前进,3保留)
            uint8_t brake : 1;            // BIT2: 刹车开关(1=开)
            uint8_t reserved_bit3 : 1;    // BIT3: 保留
            uint8_t reserved_bits47 : 4;  // BIT4-7: 保留
        } switch_status;

        // Byte 4-7: 保留
        uint8_t reserve4;
        uint8_t reserve5;
        uint8_t reserve6;
        uint8_t reserve7;
    } fields;

    uint8_t bytes[8];
};

union can_EPS_cmd_union
{
    struct
    {
        uint8_t control_mode;   // 控制方式
        uint8_t reserve1;       // 保留
        uint8_t reserve2;       // 保留
        uint8_t angle_h;        // 角度高位
        uint8_t angle_l;        // 角度低位
        uint8_t center_cmd;     // 角度对中指令
        uint8_t angular_speed;  // 角速度控制
        uint8_t checksum;       // 异或校验
    } fields;

    uint8_t bytes[8];
};

#pragma pack(pop)

struct can_MCU_cmd
{
    can_MCU_cmd_union data;

    // 根据报文信息ID (0x0CF1011E，其中011E的01改为设备ID)
    static constexpr uint32_t CMD_ID = 0x0CF1051E;
    static constexpr bool EXT_FLAG = true;
    static constexpr bool RTR_FLAG = false;

    // 构造函数自动初始化
    can_MCU_cmd()
    {
        memset(&data, 0, sizeof(data));
        // 初始化默认值
        data.fields.can_enabled = 0;               // 默认不使能
        data.fields.switch_status.cmd_status = 0;  // 默认空挡
        data.fields.switch_status.brake = 1;       // 默认刹车
    }

    // 设置使能状态
    void setEnabled(bool en) { data.fields.can_enabled = en ? 1 : 0; }

    // 设置档位 (0空挡,1后退,2前进,3保留)
    void setGear(uint8_t gear)
    {
        data.fields.switch_status.cmd_status = gear & 0x03;  // 只保留低2位
    }

    // 设置刹车状态
    void setBrake(bool brake_on) { data.fields.switch_status.brake = brake_on ? 1 : 0; }

    // 设置转速 (0-6000rpm)
    void setRPM(uint16_t rpm)
    {
        // 限制转速范围0-6000
        uint16_t clamped_rpm = std::clamp(rpm, static_cast<uint16_t>(0), static_cast<uint16_t>(6000));
        data.fields.speed_low = clamped_rpm & 0xFF;          // 低字节
        data.fields.speed_high = (clamped_rpm >> 8) & 0xFF;  // 高字节
    }

    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        memcpy(frame.data, data.bytes, 8);
        frame.dlc = 8;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;

        // LOG_INFO("MCU frame : ");
        // for (int i = 0; i < 8; ++i)
        // {
        //     LOG_INFO("0X%x ", frame.data[i]);
        // }
        // LOG_INFO("");

        return frame;
    }
};

struct can_EPS_cmd
{
    can_EPS_cmd_union data;

    static constexpr uint32_t CMD_ID = 0x469;
    static constexpr bool EXT_FLAG = false;
    static constexpr bool RTR_FLAG = false;

    can_EPS_cmd()
    {
        memset(&data, 0, sizeof(data));
        data.fields.control_mode = 0x20;
    }

    void setAngle(float angle)
    {
        // angle公式：receive[3]*256+receive[4]-1024
        int16_t raw = static_cast<int16_t>(angle * 7.0f + 1024.0f);
        data.fields.angle_h = (raw >> 8) & 0xFF;
        data.fields.angle_l = raw & 0xFF;
    }

    void setCenterCmd(uint8_t cmd) { data.fields.center_cmd = cmd; }

    void setAngularSpeed(uint16_t angle_speed)
    {
        uint8_t value = angle_speed / 6;
        if (value < 20)
            value = 20;
        else if (value > 250)
            value = 250;
        data.fields.angular_speed = static_cast<uint8_t>(value);
    }

    void pack()
    {
        // 计算校验：Byte0 ~ Byte6的异或
        uint8_t xor_sum = 0;
        for (int i = 0; i < 7; ++i)
        {
            xor_sum ^= data.bytes[i];
        }
        data.fields.checksum = xor_sum;
    }

    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        memcpy(frame.data, data.bytes, 8);
        frame.dlc = 8;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        return frame;
    }
};

// CAN指令使能信号 (0x210)
struct can_VCU_enable_cmd
{
    static constexpr uint32_t CMD_ID = 0x210;
    static constexpr bool EXT_FLAG = false;
    static constexpr bool RTR_FLAG = false;

    int8_t data[8]{};

    // 设置CAN指令使能
    void setCANEnable(bool enable)
    {
        data[0] = enable ? 1 : 0;
        // 清零其他字节
        memset(&data[1], 0, 7);
    }

    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        std::copy(std::begin(data), std::end(data), frame.data);
        frame.dlc = 8;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        return frame;
    }
};

// 电机控制指令1 (0x211) - 控制M1~M7
struct can_VCU_motor1_cmd
{
    static constexpr uint32_t CMD_ID = 0x211;
    static constexpr bool EXT_FLAG = false;
    static constexpr bool RTR_FLAG = false;

    int8_t data[8]{};

    // 设置Byte0
    void setByte0(int8_t direction) { data[0] = std::clamp(direction, static_cast<int8_t>(0), static_cast<int8_t>(2)); }

    // 设置Byte1
    void setByte1(int8_t value) { data[1] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte2
    void setByte2(int8_t value) { data[2] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte3
    void setByte3(int8_t value) { data[3] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte4
    void setByte4(int8_t value) { data[4] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte5
    void setByte5(int8_t value) { data[5] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte6
    void setByte6(int8_t value) { data[6] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte7
    void setByte7(int8_t value) { data[7] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        std::copy(std::begin(data), std::end(data), frame.data);
        frame.dlc = 8;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        return frame;
    }
};

// 电机控制指令2 (0x212) - 控制M8和LED输出
struct can_VCU_motor2_cmd
{
    static constexpr uint32_t CMD_ID = 0x212;
    static constexpr bool EXT_FLAG = false;
    static constexpr bool RTR_FLAG = false;

    int8_t data[8]{};

    // 设置Byte0
    void setByte0(int8_t value) { data[0] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte1
    void setByte1(int8_t value) { data[1] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte2
    void setByte2(int8_t value) { data[2] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte3
    void setByte3(int8_t value) { data[3] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte4
    void setByte4(int8_t value) { data[4] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte5
    void setByte5(int8_t value) { data[5] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte6
    void setByte6(int8_t value) { data[6] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    // 设置Byte7
    void setByte7(int8_t value) { data[7] = std::clamp(value, static_cast<int8_t>(-100), static_cast<int8_t>(100)); }

    CANFrame toFrame() const
    {
        CANFrame frame;
        frame.id = CMD_ID;
        std::copy(std::begin(data), std::end(data), frame.data);
        frame.dlc = 8;
        frame.ext = EXT_FLAG;
        frame.rtr = RTR_FLAG;
        return frame;
    }
};

extern can_MCU_cmd mcu_cmd;
extern can_EPS_cmd eps_cmd;
extern can_VCU_enable_cmd vcu_enable_cmd;
extern can_VCU_motor1_cmd vcu_motor1_cmd;
extern can_VCU_motor2_cmd vcu_motor2_cmd;
extern can_BMS_query_0x100 bms_query_0x100;
extern can_BMS_query_0x101 bms_query_0x101;

#endif
