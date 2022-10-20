#include <QApplication>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <random>
#include <csignal>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
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
inline const std::string LoadModeJit{"Jit"};

bool selectedLoadModeIsSupported(std::string_view mode) {
    std::array<std::string, 2> allowedModes{LoadModeNow, LoadModeJit};
    return std::ranges::find(allowedModes, mode) == std::ranges::end(allowedModes);
}

std::vector<RGB> generateColors(std::size_t N) {
    std::vector<RGB> colors{};
    std::uniform_real_distribution<double> distribution{1, 255};
    std::mt19937 random(std::chrono::system_clock::now().time_since_epoch().count());
    colors.reserve(N);
    for (auto count{0u}; count < N; ++count) {
        colors.emplace_back(RGB{distribution(random),
                                distribution(random),
                                distribution(random)});
    }
    return colors;
}

template<std::ranges::input_range Range>
auto getLoader(Range &&files, std::string_view mode) {
    if (mode == LoadModeNow)
        return std::make_unique<io::CloudLoader<PointType>>(files, io::immediateLoad);
    else if(mode == LoadModeJit)
        return std::make_unique<io::CloudLoader<PointType>>(files, io::jitLoad);
    throw std::runtime_error("Unknown loader type");
}

template<typename PointType>
void drawCloudToScreen(typename pcl::PointCloud<PointType>::Ptr cloud, const RGB &color, Viewer &viewer) {
    const auto [r, g, b]{color};
    ColorHandler colorHandler{cloud, r, g, b};
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();
    viewer.addPointCloud(cloud, colorHandler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    std::cout << "Loaded cloud  with [" << cloud->size() << "] points" << std::endl;
}

auto main(int argc, char **argv) -> int {

    std::signal(SIGSEGV, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGABRT, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGILL, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGFPE, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGPIPE, LocalizationToolkit::program::printBacktraceAndExitHandler);
    std::signal(SIGTERM, LocalizationToolkit::program::printBacktraceAndExitHandler);

    QApplication qApplication(argc, argv);

    boost::program_options::variables_map programOptions{};
    boost::program_options::options_description programOptionsDescriptions{
            "Walk into a directory and and display PCD and PLY clouds. Program usage:", 1024, 512};

    programOptionsDescriptions.add_options()("help,h", "Print out how to use the program")(
            "directory,d", boost::program_options::value<std::string>()->default_value(""),
            "Path of the directory to traverse")
            ("load,l", boost::program_options::value<std::string>()->default_value("jit"),
             "Load the cloud now or just in time");
    auto parsedProgramOptions{
            boost::program_options::command_line_parser(argc, argv).options(programOptionsDescriptions).run()};
    boost::program_options::store(parsedProgramOptions, programOptions);
    const auto isHelpRequested{programOptions.count("help") > 0};
    if (isHelpRequested) {
        std::cout << programOptionsDescriptions << std::endl;
        return EXIT_SUCCESS;
    }
    boost::program_options::notify(programOptions);

    const auto directoryToWalkPathAsString{programOptions["directory"].as<std::string>()};
    const auto loadMode{programOptions["load"].as<std::string>()};
    if (selectedLoadModeIsSupported(loadMode)) {
        std::clog << "Unknown loading mode [" << loadMode << "]" << std::endl;
        return EXIT_FAILURE;
    }

    std::filesystem::path directoryToWalkPath{};
    if (directoryToWalkPathAsString.empty()) {
        const auto selectedDirectoryAsString{io::selectDirectoryFromDialog("Select a directory to walk through")};
        if (selectedDirectoryAsString.empty()) {
            std::clog << "No directory has been selected." << std::endl;
            return EXIT_FAILURE;
        }
        directoryToWalkPath = std::filesystem::path(selectedDirectoryAsString);
    } else {
        directoryToWalkPath = std::filesystem::path(directoryToWalkPathAsString);
    }

    QCoreApplication::processEvents();
    const auto supportedExtensions{std::vector<std::filesystem::path>{PCD_EXTENSION, PLY_EXTENSION}};
    const auto &[succeed, files] {io::directoryWalk(directoryToWalkPath, supportedExtensions)};
    if (!succeed) {
        std::clog << "Could not open the directory [" << directoryToWalkPath << "]" << std::endl;
        return EXIT_FAILURE;
    }
    if (files.empty()) {
        std::clog << "Directory [" << directoryToWalkPath << "] does not contains any supported file." << std::endl;
        return EXIT_FAILURE;
    }
    if (io::checkIfAllFilePathsExist(files)) {
        std::clog << "Files are missing from [" << directoryToWalkPath << "]." << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<RGB> colors{generateColors(files.size())};
    std::unique_ptr<io::CloudLoader<PointType>> cloudLoader{getLoader(files, loadMode)};

    Viewer viewer{"PclWalkerViewer"};
    viewer.setSize(1280, 1024);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(0.0f, 0.0f, 0.0f);
    viewer.addCoordinateSystem(100.0f, 0.0f, 0.0f, 0.0f, "FrameOrigin");
    viewer.registerKeyboardCallback([&cloudLoader, &colors, &viewer](const auto &event) {
        if (!event.keyDown())
            return;
        if (event.getKeySym() == "Right") {
            const auto [hasChanged, index, cloud]{cloudLoader->next()};
            if (!hasChanged) {
                return;
            }
            drawCloudToScreen<PointType>(cloud, colors[index], viewer);
        } else if (event.getKeySym() == "Left") {
            const auto [hasChanged, index, cloud]{cloudLoader->previous()};
            if (!hasChanged) {
                return;
            }
            drawCloudToScreen<PointType>(cloud, colors[index], viewer);
        }
    });

    const auto [index, cloud]{cloudLoader->current()};
    drawCloudToScreen<PointType>(cloud, colors[index], viewer);

    while (!viewer.wasStopped()) {
        QCoreApplication::processEvents();
        viewer.spinOnce();
    }

    return EXIT_SUCCESS;
}
