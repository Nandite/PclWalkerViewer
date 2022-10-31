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

#include <iterator>

namespace algorithm {

    /**
     * Range based implementation of the std::accumulate algorithm.
     * @tparam I Input iterator type.
     * @tparam S Sentinel of the Input iterator type.
     * @tparam Value Type of the value to accumulate.
     * @tparam Op Accumulation operator type.
     * @param begin Begin of the range.
     * @param end End of the range.
     * @param init Initial value of the accumulator.
     * @param op accumulation operator.
     * @return The accumulator content.
     */
    template<std::input_iterator I, std::sentinel_for<I> S,
            typename Value, typename Op = std::plus<>>
    Value accumulate(I begin, S end, Value init, Op op = Op{}) {
        for (; begin != end; std::advance(begin, 1)) {
            init = op(std::move(init), *begin);
        }
        return init;
    }
}