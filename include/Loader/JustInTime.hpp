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

namespace pwv::io {

    /**
     * Open and load clouds on the go. This loader saves memory as the cloud are only opened
     * when needed through the CloudLoaderInterface methods. However, it incurs a time penalty
     * as the file have to be opened and the point cloud loaded before returning to the caller.
     */
    template<typename PointType>
    class JustInTime : public Loader<PointType> {
        using LoaderType = Loader<PointType>;
    public:

        /**
         * @tparam Range A range that satisfies input_range.
         * @param range A Range of path to load.
         */
        template<std::ranges::input_range Range>
        explicit JustInTime(const Range &range) : Loader<PointType>(range) {}

        /**
        * @return The current indexed point cloud.
        */
        inline LoadResult<PointType> current() override {
            const auto index{(*LoaderType::safeIndex)()};
            const auto &file{LoaderType::files[index]};
            return {false, index, file, LoaderType::openAndLoadCloudFile(file)};
        }

        /**
         * @return Load the next point cloud from the corresponding file.
         */
        inline LoadResult<PointType> next() override {
            const auto [hasChanged, index] {++(*LoaderType::safeIndex)};
            const auto &file{LoaderType::files[index]};
            if (hasChanged) {
                const auto cloud{LoaderType::openAndLoadCloudFile(file)};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }

        /**
         * @return Load the previous point cloud from the corresponding file.
         */
        inline LoadResult<PointType> previous() override {
            const auto [hasChanged, index]{--(*LoaderType::safeIndex)};
            const auto &file{LoaderType::files[index]};
            if (hasChanged) {
                const auto cloud{LoaderType::openAndLoadCloudFile(file)};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }
    };
}