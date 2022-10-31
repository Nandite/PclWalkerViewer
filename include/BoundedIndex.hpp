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

#include <concepts>
#include <limits>
#include <stdexcept>

namespace pwv {

    /**
    * A bounded integral index class [min, max] providing post increment and decrement operators.
    * @tparam integral The underlying integral type of the index.
    */
    template<std::integral integral>
    class BoundedIndex {
    public:
        explicit BoundedIndex(const integral init, const integral step = 1,
                              const integral min = std::numeric_limits<integral>::min(),
                              const integral max = std::numeric_limits<integral>::max()) :
                lowerBound(min), upperBound(max), step(step), index(min) {
            if (init < min || init > max)
                throw std::runtime_error("init must be > or equals to min and < or equals to max");
            if (max < min)
                throw std::runtime_error("min must be > or equals to max");
        }

        inline integral operator()() const {
            return index;
        }

        std::tuple<bool, integral> operator++() {
            if (upperBound - step < index)
                return {false, upperBound};
            index += step;
            return {true, index};
        }

        std::tuple<bool, integral> operator--() {
            if (lowerBound + step > index)
                return {false, lowerBound};
            index -= step;
            return {true, index};
        }

    private:
        const integral lowerBound{};
        const integral upperBound{};
        const integral step{};
        integral index{};
    };
}