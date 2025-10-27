#include "extractor/simulator.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    spdlog::set_pattern("%^" + PREFIX + "%$ [%^%l%$] %v");

    const std::string USAGE =
    "Usage:\n"
    "  ./go2_recorder --mode <high|vicon|go2odometry> --model <model_path> --storage <storage_path>\n"
    "Defaults:\n"
    "  --mode high\n"
    "  --model model/scene.xml\n"
    "  --storage test.npy\n";

    // Default values
    std::string mode = "high";
    std::string model_path = "model/scene.xml";
    std::string storage_path = "test.npy";

    if (argc > 7) {
        spdlog::error("Too many arguments provided.");
        std::cout << USAGE << std::endl;
        return 1;
    }

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            model_path = argv[++i];
        } else if (arg == "--storage" && i + 1 < argc) {
            storage_path = argv[++i];
        }
    }

    // Check mode validity
    if (mode != "high" && mode != "vicon" && mode != "go2odometry") {
        spdlog::error("Invalid mode '{}'. Valid options: high, vicon, go2odometry", mode);
        return 1;
    }

    // Map string to ExtractorMode enum
    ExtractorMode extractor_mode = ExtractorMode::HIGHSTATE;
    if (mode == "vicon") extractor_mode = ExtractorMode::VICON;
    else if (mode == "go2odometry") extractor_mode = ExtractorMode::GO2_ODOMETRY;

    Simulator simulator(extractor_mode, storage_path, model_path);
    return 0;
}