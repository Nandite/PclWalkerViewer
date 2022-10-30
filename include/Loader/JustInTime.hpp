#pragma once

#include "Loader.hpp"

namespace io {
    template<typename PointType>
    class JustInTime : public Loader<PointType> {
        using LoaderType = Loader<PointType>;
    public:

        /**
         * @tparam Range A range that satisfies input_range.
         * @param range A Range of path to load.
         */
        template<std::ranges::input_range Range>
        explicit JustInTime(const Range &range) : Loader<PointType>(range) {}

        /**
        * @return The current indexed point cloud.
        */
        inline LoadResult<PointType> current() override {
            const auto index{(*LoaderType::safeIndex)()};
            const auto &file{LoaderType::files[index]};
            return {false, index, file, LoaderType::openAndLoadCloudFile(file)};
        }

        /**
         * @return Load the next point cloud from the corresponding file.
         */
        inline LoadResult<PointType> next() override {
            const auto [hasChanged, index] {++(*LoaderType::safeIndex)};
            const auto &file{LoaderType::files[index]};
            if (hasChanged) {
                const auto cloud{LoaderType::openAndLoadCloudFile(file)};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }

        /**
         * @return Load the previous point cloud from the corresponding file.
         */
        inline LoadResult<PointType> previous() override {
            const auto [hasChanged, index]{--(*LoaderType::safeIndex)};
            const auto &file{LoaderType::files[index]};
            if (hasChanged) {
                const auto cloud{LoaderType::openAndLoadCloudFile(file)};
                return {true, index, file, cloud};
            }
            return {false, index, file, {}};
        }
    };
}