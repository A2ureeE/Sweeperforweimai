#include "mc/control_cache.hpp"

ControlCache control_cache;

void ControlCache::update(const sweeper_interfaces::msg::McCtrl& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_msg_ = msg;
    last_update_time_ = std::chrono::steady_clock::now();
    has_data_ = true;
}

bool ControlCache::get(sweeper_interfaces::msg::McCtrl& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) return false;

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_).count() > 100) return false;

    msg = latest_msg_;
    return true;
}

sweeper_interfaces::msg::McCtrl get_safe_control()
{
    sweeper_interfaces::msg::McCtrl msg;
    if (!control_cache.get(msg))
    {
        msg.brake = 1;
        msg.gear = 0;
        msg.rpm = 0;
        msg.angle = 0;
        msg.angle_speed = 120;

        msg.sweep = false;
        msg.sweep_mode = 0;  // 默认标准模式

        // 各清扫部件独立控制默认值
        msg.enable_main_brush = true;
        msg.enable_vacuum = true;
        msg.enable_dust_shake = false;
        msg.enable_main_brush_pole = true;
        msg.enable_flap_pole = true;
        msg.enable_side_brush = true;
        msg.enable_water_pump = true;
    }
    return msg;
}
