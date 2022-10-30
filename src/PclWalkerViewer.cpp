#include <iostream>
#include <random>
#include <csignal>
#include <ranges>
#include <algorithm>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "FileAndDirectory.hpp"
#include "Backtrace.hpp"
#include "CloudLoader.hpp"

#define PLY_EXTENSION ".ply"
#define PCD_EXTENSION ".pcd"

using RGB = std::tuple<double, double, double>;
using PointType = pcl::PointXYZ;
using ColorHandler = pcl::visualization::PointCloudColorHandlerCustom<PointType>;
using Viewer = pcl::visualization::PCLVisualizer;

inline const std::string LoadModeNow{"now"};
inline const std::string LoadModeJit{"jit"};

/**
 *
 * @param mode
 * @return
 */
bool selectedLoadModeIsSupported(std::string_view mode) {
    std::array<std::string, 2> allowedModes{LoadModeNow, LoadModeJit};
    return std::ranges::find(allowedModes, mode) == std::ranges::end(allowedModes);
}

template<typename RandomEngine>
RGB generateColor(RandomEngine &&engine) {
    std::uniform_real_distribution<double> distribution{1, 255};
    return {distribution(engine),
            distribution(engine),
            distribution(engine)};
}

/**
 *
 * @param N
 * @return
 */
template<typename RandomEngine>
std::vector<RGB> generateColors(const std::size_t N, RandomEngine &&engine) {
    std::vector<RGB> colors{N};
    std::uniform_real_distribution<double> distribution{1, 255};
    std::ranges::generate(std::begin(colors), std::end(colors), [&engine, &distribution]() -> RGB {
        return {distribution(engine),
                distribution(engine),
                distribution(engine)};
    });
    return colors;
}

/**
 *
 * @tparam Args
 * @param mode
 * @param args
 * @return
 */
template<typename ... Args>
auto getLoader(std::string_view mode, Args &&... args) {
    if (mode == LoadModeNow)
        return std::make_unique<io::CloudLoader<PointType>>(std::forward<Args>(args)..., io::immediateLoad);
    else if (mode == LoadModeJit)
        return std::make_unique<io::CloudLoader<PointType>>(std::forward<Args>(args)..., io::jitLoad);
    throw std::runtime_error("Unknown loader type");
}

/**
 *
 * @tparam PointType
 * @param cloud
 * @param color
 * @param viewer
 */
template<typename PointType>
void drawCloudToScreen(typename pcl::PointCloud<PointType>::Ptr cloud, const RGB &color, Viewer &viewer) {
    const auto [r, g, b]{color};
    ColorHandler colorHandler{cloud, r, g, b};
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(cloud, colorHandler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
}

auto main(int argc, char **argv) -> int {

    std::signal(SIGSEGV, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGABRT, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGILL, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGFPE, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGPIPE, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGTERM, LocalizationToolkit::program::printBacktraceAndExitHandler);

    boost::program_options::variables_map programOptions{};
    boost::program_options::options_description programOptionsDescriptions{
            "Walk into a directory and and display PCD and PLY clouds. Program usage:", 1024, 512};

    programOptionsDescriptions.add_options()("help,h", "Print out how to use the program")(
            "directory,d", boost::program_options::value<std::string>()->required(),
            "Path of the directory to traverse")
            ("load,l", boost::program_options::value<std::string>()->default_value("jit"),
             "Load the cloud now or just in time (jit or now)");
    auto parsedProgramOptions{
            boost::program_options::command_line_parser(argc, argv).options(programOptionsDescriptions).run()};
    boost::program_options::store(parsedProgramOptions, programOptions);
    const auto isHelpRequested{programOptions.count("help") > 0};
    if (isHelpRequested) {
        std::cout << programOptionsDescriptions << std::endl;
        return EXIT_SUCCESS;
    }
    boost::program_options::notify(programOptions);

    const auto loadMode{boost::algorithm::to_lower_copy(programOptions["load"].as<std::string>())};
    if (selectedLoadModeIsSupported(loadMode)) {
        std::clog << "Unknown loading mode [" << loadMode << "]" << std::endl;
        return EXIT_FAILURE;
    }

    const auto directoryToWalkPath{std::filesystem::path{programOptions["directory"].as<std::string>()}};
    const auto supportedExtensions{std::vector<std::filesystem::path>{PCD_EXTENSION, PLY_EXTENSION}};
    const auto &[succeed, files] {io::directoryWalk(directoryToWalkPath, supportedExtensions)};
    if (!succeed) {
        std::clog << "[" << directoryToWalkPath << "] is not a directory or does not exists" << std::endl;
        return EXIT_FAILURE;
    }
    if (files.empty()) {
        std::clog << "Directory [" << directoryToWalkPath << "] does not contains any supported file." << std::endl;
        return EXIT_FAILURE;
    }
    if (!io::checkIfAllFilePathsExist(files)) {
        // TODO : insert time since file load
        std::clog << "Walked files are missing from [" << directoryToWalkPath << "]." << std::endl;
        return EXIT_FAILURE;
    }

    std::mt19937 random(std::chrono::system_clock::now().time_since_epoch().count());
    auto colors{generateColors(files.size(), random)};
    auto cloudLoader{getLoader(loadMode, files)};

    Viewer viewer{"Pcl Walker Viewer"};
    viewer.setSize(1280, 1024);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(0.0f, 0.0f, 0.0f);
    viewer.addCoordinateSystem(100.0f, 0.0f, 0.0f, 0.0f, "Origin");
    viewer.registerKeyboardCallback([&cloudLoader, &colors, &random, &viewer](const auto &event) {
        if (!event.keyDown())
            return;
        if (event.getKeySym() == "Right") {
            const auto &result{cloudLoader->next()};
            if (!result.invalidated) {
                return;
            }
            std::cout << "Loaded [" << result.path << "] with [" << result.cloud->size() << "] point(s)" << std::endl;
            drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);
        } else if (event.getKeySym() == "Left") {
            const auto result{cloudLoader->previous()};
            if (!result.invalidated) {
                return;
            }
            std::cout << "Loaded [" << result.path << "] with [" << result.cloud->size() << "] point(s)" << std::endl;
            drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);
        } else if (event.getKeySym() == "d") {
            const auto result{cloudLoader->current()};
            colors[result.index] = generateColor(random);
            drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);
        }
    });

    const auto result{cloudLoader->current()};
    drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);

    viewer.spin();

    return EXIT_SUCCESS;
}
