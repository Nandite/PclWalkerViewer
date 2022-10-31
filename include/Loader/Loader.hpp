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

#pragma once

#include <filesystem>
#include <concepts>
#include <pcl/impl/point_types.hpp>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace io {

    inline static const std::string PLY_EXTENSION{".ply"};
    inline static const std::string PCD_EXTENSION{".pcd"};

    /**
     * @return Get the list of cloud file extensions supported by the application.
     */
    inline static std::vector<std::filesystem::path> getSupportedFileExtensions() {
        return {PLY_EXTENSION, PCD_EXTENSION};
    }

    /**
     * A bounded integral index class [min, max] providing post incrementation and decrementation operators.
     * @tparam integral The underlying integral type of the index.
     */
    template<std::integral integral>
    class BoundedIndex {
    public:
        explicit BoundedIndex(integral min = std::numeric_limits<integral>::min(),
                              integral max = std::numeric_limits<integral>::max()) :
                lowerBound(min), upperBound(max), index(min) {
            static_assert(max >= min, "min mus be > or equals to max");
        }

        inline integral operator()() const {
            return index;
        }

        std::tuple<bool, integral> operator++() {
            if (index >= upperBound)
                return {false, upperBound};
            return {true, ++index};
        }

        std::tuple<bool, integral> operator--() {
            if (index <= lowerBound)
                return {false, lowerBound};
            return {true, --index};
        }

    private:
        integral index{};
        const integral lowerBound{};
        const integral upperBound{};
    };

    template<typename PointType>
    struct LoadResult {

        LoadResult(const bool invalidated, const size_t index, std::filesystem::path path,
                   const typename pcl::PointCloud<PointType>::Ptr &cloud) : invalidated(invalidated), index(index),
                                                                            path(std::move(path)), cloud(cloud) {}

        const bool invalidated{};
        const std::size_t index{};
        const std::filesystem::path path{};
        const typename pcl::PointCloud<PointType>::Ptr cloud{};
    };

    template<typename PointType>
    class CloudLoaderInterface {
    public:

        /**
         * @return the current point cloud.
         */
        virtual LoadResult<PointType> current() = 0;

        /**
         * @return the next point cloud.
         */
        virtual LoadResult<PointType> next() = 0;

        /**
         * @return the previous point cloud.
         */
        virtual LoadResult<PointType> previous() = 0;

        virtual ~CloudLoaderInterface() = default;
    };

    template<typename PointType>
    class Loader : public CloudLoaderInterface<PointType> {
        using SafeIndexType = BoundedIndex<std::size_t>;
    protected:

        /**
         * Open and load a cloud from a ply file.
         * @param file path of the file to open.
         * @return a cloud with the content of the file.
         */
        static typename pcl::PointCloud<PointType>::Ptr openAndLoadPlyFile(const std::filesystem::path & file) {
            typename pcl::PointCloud<PointType>::Ptr cloud{new pcl::PointCloud<PointType>};
            pcl::io::loadPLYFile(file.string(), *cloud);
            return cloud;
        }

        /**
         * Open and load a cloud from a pcd file.
         * @param file path of the file to open.
         * @return a cloud with the content of the file.
         */
        static typename pcl::PointCloud<PointType>::Ptr openAndLoadPcdFile(const std::filesystem::path &file) {
            typename pcl::PointCloud<PointType>::Ptr cloud{new pcl::PointCloud<PointType>};
            pcl::io::loadPCDFile(file.string(), *cloud);
            return cloud;
        }

        /**
         * Open and load a cloud. The type of file to load is inferred by the extension.
         * @param file path of the file to open (ply or pcd).
         * @return a cloud with the content of the file.
         */
        static typename pcl::PointCloud<PointType>::Ptr
        openAndLoadCloudFile(const std::filesystem::path &file) noexcept(false) {
            typename pcl::PointCloud<PointType>::Ptr cloud{};
            if(!std::filesystem::exists(file))
            {
                std::clog << "[" << file << "] does not exists on the filesystem." << std::endl;
                return {};
            }
            const auto extension{file.extension()};
            if (extension == PLY_EXTENSION)
                return openAndLoadPlyFile(file);
            else if (extension == PCD_EXTENSION)
                return openAndLoadPcdFile(file);
            throw std::runtime_error("Unsupported cloud file extension.");
        }

        /**
         * Given a range of file, open and load all the clouds and store them in a sequence.
         * @tparam Range A range meeting the input_range concept and containing std::filesystem::path.
         * @param files The range of file to open and load.
         * @return a sequence of cloud each one containing the data of a file of the range. The clouds are
         * ordered according to the order of the range of files.
         */
        template<std::ranges::input_range Range>
        static std::vector<typename pcl::PointCloud<PointType>::Ptr>
        openAndLoadCloudFromPaths(Range &&files) noexcept(false) {
            std::vector<typename pcl::PointCloud<PointType>::Ptr> clouds{};
            std::ranges::for_each(files, [&clouds](const auto &file) {
                clouds.push_back(openAndLoadCloudFile(file));
            });
            return clouds;
        }

    public:
        template<std::ranges::input_range Range>
        explicit Loader(const Range &range) :
                safeIndex(std::make_unique<SafeIndexType>(0, std::ranges::size(range) - 1)) {
            std::ranges::copy(range, std::back_inserter(files));
            if (!allFilesFromRangeExist(files)) {
                std::clog << "File(s) from the range is/are are not on the filesystem anymore." << std::endl;
            }
        }

        virtual ~Loader() = default;

    protected:
        std::vector<std::filesystem::path> files{};
        std::unique_ptr<SafeIndexType> safeIndex{nullptr};
    };

}