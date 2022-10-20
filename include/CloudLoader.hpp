#pragma once
#include "Loader/Jit.hpp"
#include "Loader/Immediate.hpp"

namespace io
{
    struct NowLoadT{ explicit NowLoadT() = default; };
    struct JitLoadT{ explicit JitLoadT() = default; };
    constexpr NowLoadT  immediateLoad  = NowLoadT();
    constexpr JitLoadT  jitLoad  = JitLoadT();

    template<typename PointT>
    class CloudLoader : public LoadInterface<PointT> {
    public:

        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, NowLoadT)  : pImpl(std::make_unique<Immediate<PointT>>(range)) {}
        template<std::ranges::input_range Range>
        CloudLoader(const Range &range, JitLoadT)  : pImpl(std::make_unique<Jit<PointT>>(range)) {}
        CloudLoader(CloudLoader const&) = delete;
        CloudLoader& operator=(CloudLoader const&) = delete;

        std::tuple<std::size_t, typename pcl::PointCloud<PointT>::Ptr> current() override {
            return pImpl->current();
        }

        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> next() override {
            return pImpl->next();
        }

        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> previous() override {
            return pImpl->previous();
        }

    private:
        std::unique_ptr<LoadInterface<PointT>> pImpl{nullptr};
    };
}