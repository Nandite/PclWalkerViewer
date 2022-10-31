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