#include "mc/timer_tasks.hpp"

#include "logger/logger.h"
#include "mc/can_struct.h"
#include "mc/control_cache.hpp"

// 定时器回调：MCU 控制
void mcuTimerTask(CANDriver &canctl)
{
    auto msg = get_safe_control();
    mcu_cmd.setEnabled(true);
    mcu_cmd.setGear(msg.gear);
    mcu_cmd.setRPM(msg.rpm);
    mcu_cmd.setBrake(msg.brake);
    canctl.sendFrame(mcu_cmd.toFrame());
    // LOG_INFO("mcuTimerTask");
    // LOG_INFO("msg.gear %d", msg.gear);
    // LOG_INFO("msg.brake %d", msg.brake);
    // LOG_INFO("msg.rpm %d", msg.rpm);
}

// 定时器回调：EPS 控制
void epsTimerTask(CANDriver &canctl)
{
    auto msg = get_safe_control();
    eps_cmd.setCenterCmd(0);
    eps_cmd.setAngle(msg.angle);
    eps_cmd.setAngularSpeed(msg.angle_speed);
    eps_cmd.pack();
    canctl.sendFrame(eps_cmd.toFrame());
    // LOG_INFO("epsTimerTask");
}

// 定时器回调：VCU 扫地控制
void vcuTimerTask(CANDriver &canctl)
{
    auto msg = get_safe_control();

    // 基础值定义
    int8_t start_value = 100;  // 100表示启动
    int8_t stop_value = 0;     // 0表示停止
    int8_t lower_value = -100; // -100表示下沉/下降
    int8_t lift_value = 100;   // 100表示抬升

    // 发送CAN使能指令
    vcu_enable_cmd.setCANEnable(true);
    canctl.sendFrame(vcu_enable_cmd.toFrame());

    // 决定各部件的控制状态
    bool main_brush_enabled = msg.enable_main_brush;
    bool vacuum_enabled = msg.enable_vacuum;
    bool dust_shake_enabled = msg.enable_dust_shake;
    bool main_brush_pole_enabled = msg.enable_main_brush_pole;
    bool flap_pole_enabled = msg.enable_flap_pole;
    bool side_brush_enabled = msg.enable_side_brush;
    bool water_pump_enabled = msg.enable_water_pump;

    // ===== 控制0x211报文 (电机M1-M7) =====
    vcu_motor1_cmd.setByte0(main_brush_enabled ? 1 : 0);                         // 电机M1方向 1表示正向运行，0表示停止
    vcu_motor1_cmd.setByte1(main_brush_enabled ? start_value : stop_value);      // 电机M1油门
    vcu_motor1_cmd.setByte2(vacuum_enabled ? start_value : stop_value);          // 吸尘电机
    vcu_motor1_cmd.setByte3(dust_shake_enabled ? start_value : stop_value);      // 振尘电机
    vcu_motor1_cmd.setByte4(main_brush_pole_enabled ? lower_value : lift_value); // 主刷推杆-100 100
    vcu_motor1_cmd.setByte5(flap_pole_enabled ? lower_value : lift_value);       // 前挡皮推杆电机-100 100
    vcu_motor1_cmd.setByte6(side_brush_enabled ? start_value : stop_value);      // 边刷电机
    vcu_motor1_cmd.setByte7(side_brush_enabled ? start_value : stop_value);      // 边刷电机
    canctl.sendFrame(vcu_motor1_cmd.toFrame());

    // ===== 控制0x212报文 (电机M8和预留输出) =====
    vcu_motor2_cmd.setByte0(water_pump_enabled ? start_value : stop_value); // 水泵电机 -100 100
    vcu_motor2_cmd.setByte1(0);                                             //
    vcu_motor2_cmd.setByte2(0);                                             //
    vcu_motor2_cmd.setByte3(0);                                             //
    vcu_motor2_cmd.setByte4(0);                                             //
    vcu_motor2_cmd.setByte5(0);                                             //
    vcu_motor2_cmd.setByte6(0);                                             //
    vcu_motor2_cmd.setByte7(0);                                             //
    canctl.sendFrame(vcu_motor2_cmd.toFrame());

    LOG_INFO_THROTTLE(1000,
                      "[VCU]"
                      "主刷: %s, 吸尘: %s, 振尘: %s, "
                      "主刷推杆: %s, 前挡皮推杆电机: %s, 边刷: %s, 水泵: %s",
                      main_brush_enabled ? "On" : "Off", vacuum_enabled ? "On" : "Off",
                      dust_shake_enabled ? "On" : "Off", main_brush_pole_enabled ? "Lowered" : "Lifted",
                      flap_pole_enabled ? "Lowered" : "Lifted", side_brush_enabled ? "On" : "Off",
                      water_pump_enabled ? "On" : "Off");
}

// 定时器回调：BMS 查询任务
void bmsTimerTask(CANDriver &canctl)
{
    static bool bms_initialized = false;

    // 首次运行时初始化日志
    if (!bms_initialized)
    {
        LOG_INFO("[BMS] Query task started");
        bms_initialized = true;
    }

    // 周期性发送
    canctl.sendFrame(bms_query_0x100.toFrame());
    canctl.sendFrame(bms_query_0x101.toFrame());
}

// 注册 VID 发送定时器任务
void setupVidTimer(rclcpp::Node::SharedPtr node, std::function<void()> callback)
{
    // 200ms 持续发布 VID
    static auto timer_vid = node->create_wall_timer(std::chrono::milliseconds(200), callback);

    LOG_INFO("[TIMER] VID transmit timer setup completed");
}

// 注册所有定时器任务
void setupTimers(rclcpp::Node::SharedPtr node, CANDriver &canctl)
{
    static auto timer_mcu =
        node->create_wall_timer(std::chrono::milliseconds(50), [&canctl]()
                                { mcuTimerTask(canctl); });

    static auto timer_eps =
        node->create_wall_timer(std::chrono::milliseconds(50), [&canctl]()
                                { epsTimerTask(canctl); });

    static auto timer_vcu =
        node->create_wall_timer(std::chrono::milliseconds(100), [&canctl]()
                                { vcuTimerTask(canctl); });

    static auto timer_bms =
        node->create_wall_timer(std::chrono::milliseconds(200), [&canctl]()
                                { bmsTimerTask(canctl); });

    LOG_INFO("[TIMER] All timers setup completed");
}