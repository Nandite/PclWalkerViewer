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

using RGB = std::tuple<double, double, double>;
using PointType = pcl::PointXYZ;
using ColorHandler = pcl::visualization::PointCloudColorHandlerCustom<PointType>;
using Viewer = pcl::visualization::PCLVisualizer;

inline const std::string LOAD_MODE_IMMEDIATE{"now"};
inline const std::string LOAD_MODE_JIT{"jit"};

inline const std::string NEXT_CLOUD_KEY_SYM{"Right"};
inline const std::string PREVIOUS_CLOUD_KEY_SYM{"Left"};
inline const std::string CHANGE_CLOUD_COLOR_KEY_SYM{"d"};

inline const std::string APPLICATION_NAME{"Pcl Walker Viewer"};

inline const std::string CLOUD_ID{"cloud"};

/**
 *
 * @param mode
 * @return
 */
bool selectedLoadModeIsSupported(std::string_view mode) {
    std::array<std::string, 2> allowedModes{LOAD_MODE_IMMEDIATE, LOAD_MODE_JIT};
    return std::ranges::find(allowedModes, mode) == std::ranges::end(allowedModes);
}

/**
 *
 * @tparam UniformRandomNumberGenerator
 * @param numberGenerator
 * @return
 */
template<typename UniformRandomNumberGenerator>
RGB generateColor(UniformRandomNumberGenerator &&numberGenerator) {
    std::uniform_real_distribution<double> distribution{1, 255};
    const auto r{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
    const auto g{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
    const auto b{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
    return {r, g, b};
}

/**
 *
 * @tparam UniformRandomNumberGenerator
 * @param N
 * @param numberGenerator
 * @return
 */
template<typename UniformRandomNumberGenerator>
std::vector<RGB> generateColors(const std::size_t N, UniformRandomNumberGenerator &&numberGenerator) {
    std::vector<RGB> colors{N};
    std::uniform_real_distribution<double> distribution{1, 255};
    std::ranges::generate(colors, [&numberGenerator, &distribution]() -> RGB {
        const auto r{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
        const auto g{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
        const auto b{distribution(std::forward<UniformRandomNumberGenerator>(numberGenerator))};
        return {r, g, b};
    });
    return colors;
}

/**
 * Get the appropriate Loader depending on the load mode.
 * @tparam Args The arguments types of the loader to construct.
 * @param mode The cloud loading mode desired ("jit" or "immediate")
 * @param args The arguments of the loader (forwarded to the constructor call).
 * @return An instance of CloudLoader using the requested strategy to load the clouds from the filesystem.
 */
template<typename ... Args>
auto getLoader(std::string_view mode, Args &&... args) {
    if (mode == LOAD_MODE_IMMEDIATE)
        return std::make_unique<io::CloudLoader<PointType>>(std::forward<Args>(args)..., io::immediateLoad);
    else if (mode == LOAD_MODE_JIT)
        return std::make_unique<io::CloudLoader<PointType>>(std::forward<Args>(args)..., io::jitLoad);
    throw std::runtime_error("Unknown loader type");
}

/**
 * Draw a point cloud to the screen. All previous cloud drawn are removed beforehand.
 * @tparam PointType The type of the point to draw.
 * @param cloud The cloud to draw to the screen.
 * @param color The RGB color with which the cloud is drawn.
 * @param viewer The viewer to draw the cloud into.
 */
template<typename PointType>
void drawCloudToScreen(typename pcl::PointCloud<PointType>::Ptr cloud, const RGB &color, Viewer &viewer) {
    const auto [r, g, b]{color};
    ColorHandler colorHandler{cloud, r, g, b};
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(cloud, colorHandler, CLOUD_ID);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, CLOUD_ID);
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
            ("load,l", boost::program_options::value<std::string>()->default_value(LOAD_MODE_JIT),
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
    const auto supportedExtensions{io::getSupportedFileExtensions()};
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

    Viewer viewer{APPLICATION_NAME};
    viewer.setSize(1280, 1024);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(0.0f, 0.0f, 0.0f);
    viewer.addCoordinateSystem(100.0f, 0.0f, 0.0f, 0.0f, "Origin");
    viewer.registerKeyboardCallback([&cloudLoader, &colors, &random, &viewer](const auto &event) {
        if (!event.keyDown())
            return;
        if (event.getKeySym() == NEXT_CLOUD_KEY_SYM) {
            const auto &result{cloudLoader->next()};
            if (!result.invalidated) {
                return;
            }
            std::cout << "Loaded [" << result.path << "] with [" << result.cloud->size() << "] point(s)"
                      << std::endl;
            drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);
        } else if (event.getKeySym() == PREVIOUS_CLOUD_KEY_SYM) {
            const auto result{cloudLoader->previous()};
            if (!result.invalidated) {
                return;
            }
            std::cout << "Loaded [" << result.path << "] with [" << result.cloud->size() << "] point(s)"
                      << std::endl;
            drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);
        } else if (event.getKeySym() == CHANGE_CLOUD_COLOR_KEY_SYM) {
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
