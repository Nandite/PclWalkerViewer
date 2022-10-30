#pragma once

#include "Loader.hpp"

namespace io {
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