#pragma once

#include <iterator>

namespace algorithm {
    template<std::input_iterator I, std::sentinel_for<I> S,
            typename Init, typename Op = std::plus<>>
    Init accumulate(I begin, S end, Init init, Op op = Op{}) {
        for (; begin != end; std::advance(begin, 1)) {
            init = op(std::move(init), *begin);
        }
        return init;
    }
}