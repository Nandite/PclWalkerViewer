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

#include "Loader/JustInTime.hpp"
#include "Loader/Immediate.hpp"

namespace pwv::io
{
    struct LoadNowStrategyT{ explicit LoadNowStrategyT() = default; };
    struct LoadJitStrategyT{ explicit LoadJitStrategyT() = default; };
    constexpr LoadNowStrategyT  immediateLoad {LoadNowStrategyT()};
    constexpr LoadJitStrategyT  jitLoad {LoadJitStrategyT()};

    template<typename PointType>
    class CloudLoader : public CloudLoaderInterface<PointType> {
    public:

        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, LoadNowStrategyT)  : pImpl(std::make_unique<Immediate<PointType>>(range)) {}
        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, LoadJitStrategyT)  : pImpl(std::make_unique<JustInTime<PointType>>(range)) {}
        CloudLoader(CloudLoader const&) = delete;
        CloudLoader& operator=(CloudLoader const&) = delete;

        io::LoadResult<PointType> current() override {
            return pImpl->current();
        }

        io::LoadResult<PointType> next() override {
            return pImpl->next();
        }

        io::LoadResult<PointType> previous() override {
            return pImpl->previous();
        }

    private:
        std::unique_ptr<CloudLoaderInterface<PointType>> pImpl{nullptr};
    };
}