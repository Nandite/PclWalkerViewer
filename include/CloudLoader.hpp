#pragma once
#include "Loader/JustInTime.hpp"
#include "Loader/Immediate.hpp"

namespace io
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