#pragma once
#include "Loader/JustInTime.hpp"
#include "Loader/Immediate.hpp"

namespace io
{
    struct NowLoadT{ explicit NowLoadT() = default; };
    struct JitLoadT{ explicit JitLoadT() = default; };
    constexpr NowLoadT  immediateLoad {NowLoadT()};
    constexpr JitLoadT  jitLoad {JitLoadT()};

    template<typename PointType>
    class CloudLoader : public CloudLoaderInterface<PointType> {
    public:

        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, NowLoadT)  : pImpl(std::make_unique<Immediate<PointType>>(range)) {}
        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, JitLoadT)  : pImpl(std::make_unique<JustInTime<PointType>>(range)) {}
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