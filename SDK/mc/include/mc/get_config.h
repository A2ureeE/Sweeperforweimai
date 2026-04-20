#pragma once

#include <fstream>
#include <string>
#include <iostream>
#include <chrono>
#include <random>
#include <sstream>
#include <iomanip>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

struct Config
{
    std::string can_dev;
};

bool load_config(Config &config);