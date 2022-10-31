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

#include "Loader.hpp"

namespace io {

    /**
     * Open and load all the point clouds immediately. The point clouds are then stored in memory.
     * This strategy causes more memory consumption, however, it is way faster than opening the file
     * and loading the point cloud whenever the CloudLoaderInterface's methods are called.
     */
    template<typename PointType>
    class Immediate : public Loader<PointType> {
        using PointCloud = pcl::PointCloud<PointType>;
    public:

        /**
         * @tparam Range A range that satisfies input_range.
         * @param range A Range of path to load.
         */
        template<std::ranges::input_range Range>
        explicit Immediate(const Range &range) : Loader<PointType>(range),
                                                 clouds(Loader<PointType>::openAndLoadCloudFromPaths(
                                                         Loader<PointType>::files)) {}

        /**
         * @return The current stored point cloud.
         */
        LoadResult<PointType> current() override {
            const auto index{(*Loader<PointType>::safeIndex)()};
            return {false, index, Loader<PointType>::files[index], clouds[index]};
        }

        /**
         * @return the next stored point cloud.
         */
        LoadResult<PointType> next() override {
            const auto [hasChanged, index] {++(*Loader<PointType>::safeIndex)};
            const auto file{Loader<PointType>::files[index]};
            if (hasChanged) {
                const auto cloud{clouds[index]};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }

        /**
         * @return the previous stored point cloud.
         */
        LoadResult<PointType> previous() override {
            const auto [hasChanged, index] {--(*Loader<PointType>::safeIndex)};
            const auto file{Loader<PointType>::files[index]};
            if (hasChanged) {
                const auto cloud{clouds[index]};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }

    private:
        std::vector<typename PointCloud::Ptr> clouds{};
    };
}