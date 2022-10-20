#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include "Accumulate.hpp"
#include <ranges>
#include <boost/algorithm/string/trim.hpp>

namespace io {

    std::string selectDirectoryFromDialog(const std::string& title);

    template<std::ranges::input_range Range>
    bool checkIfAllFilePathsExist(Range && range) {
        return algorithm::accumulate(std::ranges::begin(range), std::ranges::end(range), true,
                                     [](const auto v, const auto &path) {
                                         const auto exists{std::filesystem::exists(path)};
                                         if (!exists)
                                             std::clog << "File [" << path << "] does not exists" << std::endl;
                                         return std::logical_and<>()(v, exists);
                                     });
    }

    template<std::ranges::input_range Range>
    std::pair<bool, std::vector<std::filesystem::path>>
    directoryWalk(const std::filesystem::path &directory, Range &&extensions) {
        std::vector<std::filesystem::path> files{};
        if (std::filesystem::exists(directory) && std::filesystem::is_directory(directory)) {
            for (auto const &file: std::filesystem::recursive_directory_iterator(directory)) {
                const auto extension{file.path().extension()};
                if (std::filesystem::is_regular_file(file) &&
                    (std::ranges::find(extensions, extension) != std::ranges::end(extensions))) {
                    files.emplace_back(file.path().filename());
                }
            }
        } else {
            std::clog << "[" << directory << "] is not a directory or does not exists" << std::endl;
            return {false, {}};
        }
        return {true, files};
    }

} // namespace io
