#pragma once

#include <filesystem>
#include <concepts>
#include <pcl/impl/point_types.hpp>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace io {

    inline static const std::string PLY_EXTENSION{".ply"};
    inline static const std::string PCD_EXTENSION{".pcd"};
    inline static std::vector<std::filesystem::path> getSupportedFileExtensions() {
        return {PLY_EXTENSION, PCD_EXTENSION};
    }

    template<std::integral integral>
    class SafeIndex {
    public:
        explicit SafeIndex(integral min = std::numeric_limits<integral>::min(),
                           integral max = std::numeric_limits<integral>::max()) :
                lowerBound(min), upperBound(max) {}

        inline integral operator()() const {
            return index;
        }

        std::tuple<bool, integral> operator++() {
            if (index >= upperBound)
                return {false, upperBound};
            return {true, (++index)};
        }

        std::tuple<bool, integral> operator--() {
            if (index <= lowerBound)
                return {false, lowerBound};
            return {true, (--index)};
        }

    private:
        integral index{};
        const integral lowerBound{}, upperBound{};
    };

    template<typename PointType>
    struct LoadResult {

        LoadResult(const bool invalidated, const size_t index, std::filesystem::path path,
                   const typename pcl::PointCloud<PointType>::Ptr &cloud) : invalidated(invalidated), index(index),
                                                                            path(std::move(path)), cloud(cloud) {}

        const bool invalidated{};
        const std::size_t index{};
        const std::filesystem::path path{};
        const typename pcl::PointCloud<PointType>::Ptr cloud{};
    };

    template<typename PointType>
    class CloudLoaderInterface {
    public:

        /**
         * @return the current point cloud.
         */
        virtual LoadResult<PointType> current() = 0;

        /**
         * @return the next point cloud.
         */
        virtual LoadResult<PointType> next() = 0;

        /**
         * @return the previous point cloud.
         */
        virtual LoadResult<PointType> previous() = 0;

        virtual ~CloudLoaderInterface() = default;
    };

    template<typename PointType>
    class Loader : public CloudLoaderInterface<PointType> {
        using SafeIndexType = SafeIndex<std::size_t>;
    protected:
        static typename pcl::PointCloud<PointType>::Ptr openAndLoadPlyFile(const std::filesystem::path &path) {
            typename pcl::PointCloud<PointType>::Ptr cloud{new pcl::PointCloud<PointType>};
            pcl::io::loadPLYFile(path.string(), *cloud);
            return cloud;
        }

        static typename pcl::PointCloud<PointType>::Ptr openAndLoadPcdFile(const std::filesystem::path &path) {
            typename pcl::PointCloud<PointType>::Ptr cloud{new pcl::PointCloud<PointType>};
            pcl::io::loadPCDFile(path.string(), *cloud);
            return cloud;
        }

        static typename pcl::PointCloud<PointType>::Ptr
        openAndLoadCloudFile(const std::filesystem::path &file) noexcept(false) {
            typename pcl::PointCloud<PointType>::Ptr cloud{};
            const auto extension{file.extension()};
            if (extension == PLY_EXTENSION)
                return openAndLoadPlyFile(file);
            else if (extension == PCD_EXTENSION)
                return openAndLoadPcdFile(file);
            throw std::runtime_error("Unsupported cloud file extension.");
        }

        template<std::ranges::input_range Range>
        static std::vector<typename pcl::PointCloud<PointType>::Ptr>
        openAndLoadCloudFromPaths(Range &&files) noexcept(false) {
            std::vector<typename pcl::PointCloud<PointType>::Ptr> clouds{};
            std::ranges::for_each(files, [&clouds](const auto &file) {
                clouds.push_back(openAndLoadCloudFile(file));
            });
            return clouds;
        }

    public:
        template<std::ranges::input_range Range>
        explicit Loader(const Range &range) :
                safeIndex(std::make_unique<SafeIndexType>(0, std::ranges::size(range) - 1)) {
            std::ranges::copy(range, std::back_inserter(files));
        }

        virtual ~Loader() = default;

    protected:
        std::vector<std::filesystem::path> files{};
        std::unique_ptr<SafeIndexType> safeIndex{nullptr};
    };

}