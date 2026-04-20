#include "mc/get_config.h"

bool load_config(Config& config)
{
    try
    {
        std::string config_path = ament_index_cpp::get_package_share_directory("mc") + "/config/config.json";

        std::ifstream ifs(config_path);
        if (!ifs.is_open())
        {
            std::cerr << "Failed to open config file: " << config_path << std::endl;
            return false;
        }

        json j;
        ifs >> j;

        config.can_dev = j.at("can_dev").get<std::string>();

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error parsing MQTT config: " << e.what() << std::endl;
        return false;
    }
}
