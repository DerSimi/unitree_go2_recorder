#include "extractor/simulator.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    spdlog::set_pattern("%^" + PREFIX + "%$ [%^%l%$] %v");

    std::string mode = "high";
    std::string model_path = "model/scene.xml";
    std::string storage_path = "storage.npy";

    // Vicon strings
    std::string vicon_ip = "10.0.0.20";
    std::string vicon_subject = "Go2_base";

    // Usage string with dynamic defaults
    const std::string USAGE =
        "Usage:\n"
        "  ./go2_recorder --mode <high|vicon> --model <model_path> --storage <storage_path>\n"
        "Defaults:\n"
        "  --mode " +
        mode + "\n"
               "  --model " +
        model_path + "\n"
                     "  --storage " +
        storage_path + "\n"
                       "  (VICON) --vicon_ip " +
        vicon_ip + "\n"
                   "  (VICON) --vicon_subject " +
        vicon_subject + "\n";

    if (argc == 1)
    {
        std::cout << USAGE << std::endl;
        return 1;
    }

    
    if (argc > 11)
    {
        spdlog::error("Too many arguments provided.");
        std::cout << USAGE << std::endl;
        return 1;
    }

    // Parse arguments
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--mode" && i + 1 < argc)
        {
            mode = argv[++i];
        }
        else if (arg == "--model" && i + 1 < argc)
        {
            model_path = argv[++i];
        }
        else if (arg == "--storage" && i + 1 < argc)
        {
            storage_path = argv[++i];
        }
        else if (arg == "--vicon_ip" && i + 1 < argc)
        {
            vicon_ip = argv[++i];
        }
        else if (arg == "--vicon_subject" && i + 1 < argc)
        {
            vicon_subject = argv[++i];
        }
    }

    // Check mode validity
    if (mode != "high" && mode != "vicon")
    {
        spdlog::error("Invalid mode '{}'. Valid options: high, vicon", mode);
        return 1;
    }

    // Map string to ExtractorMode enum
    ExtractorMode extractor_mode = ExtractorMode::HIGHSTATE;
    if (mode == "vicon")
        extractor_mode = ExtractorMode::VICON;

    Simulator simulator(extractor_mode, storage_path, model_path, vicon_ip, vicon_subject);
    return 0;
}