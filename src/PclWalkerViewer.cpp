// Copyright (c) 2022 Papa Libasse Sow.
// https://github.com/Nandite/PclWalkerViewer
// Distributed under the MIT Software License (X11 license).
//
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of
// the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
#include "CloudLoader.hpp"
#include "BoundedIndex.hpp"

using RGB = std::tuple<double, double, double>;
using PointType = pcl::PointXYZ;
using ColorHandler = pcl::visualization::PointCloudColorHandlerCustom<PointType>;
using Viewer = pcl::visualization::PCLVisualizer;

inline const std::string LOAD_MODE_IMMEDIATE{"now"};
inline const std::string LOAD_MODE_JIT{"jit"};

inline const std::string NEXT_CLOUD_KEY_SYM{"Right"};
inline const std::string PREVIOUS_CLOUD_KEY_SYM{"Left"};
inline const std::string CHANGE_CLOUD_COLOR_KEY_SYM{"d"};
inline const std::string TOGGLE_ORIGIN_COORD_SYSTEM_KEY_SYM{"t"};
inline const std::string INCREASE_ORIGIN_COORD_KEY_SYM{"Up"};
inline const std::string DECREASE_ORIGIN_COORD_KEY_SYM{"Down"};

inline const std::string APPLICATION_NAME{"Pcl Walker Viewer"};

inline const std::string CLOUD_ID{"cloud"};
inline const std::string ORIGIN_COORD_ID{"origin"};

/**
 * Check if the selected load strategy is supported by the application.
 * @param mode The load strategy selected to test.
 * @return True if a load strategy is supported and implemented, False otherwise.
 */
bool selectedLoadModeIsSupported(std::string_view mode) {
    std::array<std::string, 2> allowedModes{LOAD_MODE_IMMEDIATE, LOAD_MODE_JIT};
    return std::ranges::find(allowedModes, mode) == std::ranges::end(allowedModes);
}

/**
 * Generate a random RGB color.
 * @tparam UniformRandomNumberGenerator type meeting the requirements of UniformRandomBitGenerator.
 * @param numberGenerator a random generator to use to generate the color.
 * @return A random RGB color object.
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
 * Randomly generate a continuous sequence of RGB colors.
 * @tparam UniformRandomNumberGenerator type meeting the requirements of UniformRandomBitGenerator.
 * @param N The number of color to generate.
 * @param numberGenerator a random generator to use to generate the color.
 * @return A sequence of random color objects.
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
        return std::make_unique<pwv::io::CloudLoader<PointType>>(std::forward<Args>(args)..., pwv::io::immediateLoad);
    else if (mode == LOAD_MODE_JIT)
        return std::make_unique<pwv::io::CloudLoader<PointType>>(std::forward<Args>(args)..., pwv::io::jitLoad);
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

std::string getKeymapDescription() {
    return "You can control the viewer using the following inputs of the keyboard:\n"
           "- [<-] [->] (Left or Right arrow) to display the previous/next cloud\n"
           "- [" + CHANGE_CLOUD_COLOR_KEY_SYM + "] to change the color of the cloud\n"
                                                "- [" + TOGGLE_ORIGIN_COORD_SYSTEM_KEY_SYM +
           "] to toggle the display of origin coordinate system\n"
           "- [" + INCREASE_ORIGIN_COORD_KEY_SYM + "] to increase the size of the origin coordinate system\n"
                                                   "- [" + DECREASE_ORIGIN_COORD_KEY_SYM +
           "] to increase the size of the origin coordinate system\n";
}

auto main(int argc, char **argv) -> int {

    boost::program_options::variables_map programOptions{};
    boost::program_options::options_description programOptionsDescriptions{
            "Walk into a directory and and display PCD and PLY clouds. Program usage:", 1024, 512};

    programOptionsDescriptions.add_options()("help,h", "Print out how to use the program")(
            "directory,d", boost::program_options::value<std::string>()->required(),
            "Path of the directory to traverse")(
            "recursive,r", boost::program_options::bool_switch()->default_value(false),
            "Traverse the directory recursively")
            ("load,l", boost::program_options::value<std::string>()->default_value(LOAD_MODE_JIT),
             "Load the cloud now or just in time (jit or now)");
    auto parsedProgramOptions{
            boost::program_options::command_line_parser(argc, argv).options(programOptionsDescriptions).run()};
    boost::program_options::store(parsedProgramOptions, programOptions);
    const auto isHelpRequested{programOptions.count("help") > 0};
    if (isHelpRequested) {
        std::cout << programOptionsDescriptions << std::endl;
        std::cout << getKeymapDescription() << std::endl;
        return EXIT_SUCCESS;
    }
    boost::program_options::notify(programOptions);

    const auto loadMode{boost::algorithm::to_lower_copy(programOptions["load"].as<std::string>())};
    if (selectedLoadModeIsSupported(loadMode)) {
        std::clog << "Unknown loading mode [" << loadMode << "]" << std::endl;
        return EXIT_FAILURE;
    }

    const auto directoryToWalkPath{std::filesystem::path{programOptions["directory"].as<std::string>()}};
    const auto recursive{programOptions["recursive"].as<bool>()};
    const auto supportedExtensions{pwv::io::getSupportedFileExtensions()};
    const auto &[succeed, files] {pwv::io::directoryWalk(directoryToWalkPath, recursive, supportedExtensions)};
    if (!succeed) {
        std::clog << "[" << directoryToWalkPath << "] is not a directory or does not exists" << std::endl;
        return EXIT_FAILURE;
    }
    if (files.empty()) {
        std::clog << "Directory [" << directoryToWalkPath << "] does not contains any supported file." << std::endl;
        return EXIT_FAILURE;
    }

    std::mt19937 random(std::chrono::system_clock::now().time_since_epoch().count());
    auto colors{generateColors(files.size(), random)};
    auto cloudLoader{getLoader(loadMode, files)};
    auto showOrigin{true};
    pwv::BoundedIndex<std::uint32_t> originSize{1, 1, 1};

    Viewer viewer{APPLICATION_NAME};
    viewer.setSize(1280, 1024);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(0.0f, 0.0f, 0.0f);
    viewer.addCoordinateSystem(originSize(), 0.0f, 0.0f, 0.0f, ORIGIN_COORD_ID);
    viewer.registerKeyboardCallback(
            [&cloudLoader, &colors, &random, &viewer, &showOrigin, &originSize](const auto &event) {
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
                } else if (event.getKeySym() == TOGGLE_ORIGIN_COORD_SYSTEM_KEY_SYM) {
                    const auto state{std::exchange(showOrigin, !showOrigin)};
                    if (state) {
                        viewer.removeCoordinateSystem(ORIGIN_COORD_ID);
                    } else {
                        viewer.addCoordinateSystem(originSize(), 0.0f, 0.0f, 0.0f, ORIGIN_COORD_ID);
                    }
                } else if (event.getKeySym() == INCREASE_ORIGIN_COORD_KEY_SYM && showOrigin) {
                    ++originSize;
                    viewer.removeCoordinateSystem(ORIGIN_COORD_ID);
                    viewer.addCoordinateSystem(originSize(), 0.0f, 0.0f, 0.0f, ORIGIN_COORD_ID);
                } else if (event.getKeySym() == DECREASE_ORIGIN_COORD_KEY_SYM && showOrigin) {
                    --originSize;
                    viewer.removeCoordinateSystem(ORIGIN_COORD_ID);
                    viewer.addCoordinateSystem(originSize(), 0.0f, 0.0f, 0.0f, ORIGIN_COORD_ID);
                }
            });

    const auto result{cloudLoader->current()};
    drawCloudToScreen<PointType>(result.cloud, colors[result.index], viewer);

    viewer.spin();

    return EXIT_SUCCESS;
}
