#pragma once

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include "Accumulate.hpp"
#include <ranges>

namespace io {

    /**
     * Check if all paths pointing to files exists on the file system.
     * @tparam Range An input range of std::path to check.
     * @return True if all files exist, False otherwise.
     */
    template<std::ranges::input_range Range>
    bool allFilesFromRangeExist(Range &&range) {
        return algorithm::accumulate(std::ranges::begin(range), std::ranges::end(range), true,
                                     [](const auto v, const auto &path) {
                                         const auto exists{std::filesystem::exists(path)};
                                         if (!exists)
                                             std::clog << "File [" << path << "] does not exists on the filesystem" << std::endl;
                                         return std::logical_and()(v, exists);
                                     });
    }

    /**
     * Walk into a directory and return all file matching a given sequence of extension.
     * Symbolic links are no considered during the walk through.
     * @tparam Range Input range type of std::path.
     * @param directory The directory to walk into.
     * @param extensions A range of extensions to check for within the directory.
     * @return a pair containing:
     * - A Flag set to True if the directory has been walked through, False otherwise.
     * - A list of file matching the provided extension sequence to look for.
     */
    template<std::ranges::input_range Range>
    std::pair<bool, std::vector<std::filesystem::path>>
    directoryWalk(const std::filesystem::path &directory, Range &&extensions) {
        std::vector<std::filesystem::path> files{};
        if (std::filesystem::exists(directory) && std::filesystem::is_directory(directory)) {
            for (auto const &file: std::filesystem::recursive_directory_iterator(directory)) {
                const auto extension{file.path().extension()};
                if (std::filesystem::is_regular_file(file) &&
                    (std::ranges::find(extensions, extension) != std::ranges::end(extensions))) {
                    files.emplace_back(file.path());
                }
            }
        } else {
            return {false, {}};
        }
        return {true, files};
    }

} // namespace io
