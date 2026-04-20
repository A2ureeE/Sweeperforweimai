#include "mc/can_driver.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

CANDriver::CANDriver() = default;

CANDriver::~CANDriver() { close(); }

bool CANDriver::open(const std::string& interface)
{
    if (running) return false;

    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        perror("socket");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        ::close(sockfd);
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("bind");
        ::close(sockfd);
        return false;
    }

    // 设置为非阻塞
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    running = true;
    receiveThread = std::thread(&CANDriver::receiveThreadFunc, this);
    return true;
}

void CANDriver::close()
{
    if (!running) return;

    running = false;
    if (receiveThread.joinable()) receiveThread.join();

    if (sockfd >= 0)
    {
        ::close(sockfd);
        sockfd = -1;
    }
}

bool CANDriver::sendFrame(const CANFrame& frame)
{
    if (!running) return false;

    struct can_frame raw_frame{};
    raw_frame.can_id = frame.id;
    if (frame.ext) raw_frame.can_id |= CAN_EFF_FLAG;
    if (frame.rtr) raw_frame.can_id |= CAN_RTR_FLAG;
    raw_frame.can_dlc = frame.dlc;
    std::memcpy(raw_frame.data, frame.data, frame.dlc);

    if (write(sockfd, &raw_frame, sizeof(raw_frame)) != sizeof(raw_frame))
    {
        perror("write");
        return false;
    }

    return true;
}

void CANDriver::setReceiveCallback(ReceiveCallback callback, void* userData)
{
    this->callback = callback;
    this->userData = userData;
}

bool CANDriver::setFilter(const std::vector<can_filter>& filters)
{
    if (!running) return false;

    filters_ = filters;
    return applyFilters();
}

bool CANDriver::addFilter(const can_filter& filter)
{
    filters_.push_back(filter);
    return applyFilters();
}

bool CANDriver::addFilters(const std::vector<can_filter>& filters)
{
    filters_.insert(filters_.end(), filters.begin(), filters.end());
    return applyFilters();
}

bool CANDriver::applyFilters()
{
    if (!running) return false;

    if (filters_.empty())
    {
        setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
        return true;
    }

    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, filters_.data(), filters_.size() * sizeof(can_filter)) < 0)
    {
        perror("setsockopt");
        return false;
    }

    return true;
}

void CANDriver::receiveThreadFunc()
{
    struct can_frame raw_frame;
    struct pollfd fds;
    fds.fd = sockfd;
    fds.events = POLLIN;

    while (running)
    {
        int ret = poll(&fds, 1, 100);  // 等待最多100ms
        if (ret < 0)
        {
            perror("poll");
            continue;
        }
        else if (ret == 0)
        {
            // 超时无数据，可继续
            continue;
        }

        ssize_t nbytes = read(sockfd, &raw_frame, sizeof(raw_frame));
        if (nbytes == sizeof(raw_frame) && callback)
        {
            CANFrame frame;
            frame.id = raw_frame.can_id & CAN_EFF_MASK;
            frame.ext = !!(raw_frame.can_id & CAN_EFF_FLAG);
            frame.rtr = !!(raw_frame.can_id & CAN_RTR_FLAG);
            frame.dlc = raw_frame.can_dlc;
            memcpy(frame.data, raw_frame.data, raw_frame.can_dlc);

            // 调用回调函数，处理接收到的帧
            callback(frame, userData);
        }
    }
}