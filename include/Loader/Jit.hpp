#pragma once

#include "Loader.hpp"

namespace io {
    template<typename PointT>
    class Jit : public Loader<PointT> {
    public:

        /**
         * @tparam Range A range that satisfies input_range.
         * @param range A Range of path to load.
         */
        template<std::ranges::input_range Range>
        explicit Jit(const Range &range) : Loader<PointT>(range) {}

        /**
        * @return The current indexed point cloud.
        */
        std::tuple<std::size_t, typename pcl::PointCloud<PointT>::Ptr> current() override {
            const auto index {(*Loader<PointT>::safeIndex)()};
            const auto &file{Loader<PointT>::files[index]};
            return {index, Loader<PointT>::openAndLoadCloudFile(file)};
        }

        /**
         * @return Load the next point cloud from the corresponding file.
         */
        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> next() override {
            const auto [hasChanged, index] {++(*Loader<PointT>::safeIndex)};
            if (hasChanged) {
                const auto &file{Loader<PointT>::files[index]};
                return {true, index, Loader<PointT>::openAndLoadCloudFile(file)};
            }
            return {false, index, {}};
        }

        /**
         * @return Load the previous point cloud from the corresponding file.
         */
        std::tuple<bool, std::size_t , typename pcl::PointCloud<PointT>::Ptr> previous() override {
            const auto [hasChanged, index]{--(*Loader<PointT>::safeIndex)};
            if (hasChanged) {
                const auto &file{Loader<PointT>::files[index]};
                return {true, index,  Loader<PointT>::openAndLoadCloudFile(file)};
            }
            return {false, index, {}};
        }
    };
}