#pragma once
#include "sweeper_interfaces/msg/mc_ctrl.hpp"
#include <mutex>
#include <chrono>

class ControlCache
{
public:
    void update(const sweeper_interfaces::msg::McCtrl &msg);
    bool get(sweeper_interfaces::msg::McCtrl &msg);

private:
    std::mutex mutex_;
    sweeper_interfaces::msg::McCtrl latest_msg_;
    std::chrono::steady_clock::time_point last_update_time_;
    bool has_data_ = false;
};

extern ControlCache control_cache;
sweeper_interfaces::msg::McCtrl get_safe_control(); // 获取带超时判断的控制指令
