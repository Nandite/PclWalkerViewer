#pragma once

#include "Loader.hpp"

namespace io {
    template<typename PointT>
    class Immediate : public Loader<PointT> {
        using PointCloud = pcl::PointCloud<PointT>;
    public:

        /**
         * @tparam Range A range that satisfies input_range.
         * @param range A Range of path to load.
         */
        template<std::ranges::input_range Range>
        explicit Immediate(const Range &range) : Loader<PointT>(range),
                                                 clouds(Loader<PointT>::openAndLoadCloudFromPaths(
                                                         Loader<PointT>::files)) {}

        /**
         * @return The current stored point cloud.
         */
        std::tuple<std::size_t, typename pcl::PointCloud<PointT>::Ptr> current() override {
            const auto index{(*Loader<PointT>::safeIndex)()};
            return {index, clouds[index]};
        }

        /**
         * @return the next stored point cloud.
         */
        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> next() override {
            const auto [hasChanged, index] {++(*Loader<PointT>::safeIndex)};
            if (hasChanged) {
                return {true, index, clouds[index]};
            }
            return {false, index, {}};
        }

        /**
         * @return the previous stored point cloud.
         */
        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> previous() override {
            const auto [hasChanged, index] {--(*Loader<PointT>::safeIndex)};
            if (hasChanged) {
                return {true, index, clouds[index]};
            }
            return {false, index, {}};
        }

    private:
        std::vector<typename PointCloud::Ptr> clouds{};
    };
}