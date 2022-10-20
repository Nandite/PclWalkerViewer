#pragma once

#include <filesystem>
#include <concepts>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define PLY_EXTENSION ".ply"
#define PCD_EXTENSION ".pcd"

namespace io {

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

    template<typename PointT>
    class LoadInterface {
    public:

        /**
         * @return the current point cloud.
         */
        virtual std::tuple<std::size_t, typename pcl::PointCloud<PointT>::Ptr> current() = 0;

        /**
         * @return the next point cloud.
         */
        virtual std::tuple<bool, std::size_t, typename pcl::PointCloud<PointT>::Ptr> next() = 0;

        /**
         * @return the previous point cloud.
         */
        virtual std::tuple<bool, std::size_t, typename pcl::PointCloud<PointT>::Ptr> previous() = 0;

        virtual ~LoadInterface() = default;
    };

    template<typename PointT>
    class Loader : public LoadInterface<PointT> {
        using SafeIndexType = SafeIndex<std::size_t>;
    protected:
        static typename pcl::PointCloud<PointT>::Ptr openAndLoadPlyFile(const std::filesystem::path &path) {
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::io::loadPLYFile(path.string(), *cloud);
            std::cout << "Loaded a Ply file with [" << cloud->size() << "] point(s)" << std::endl;
            return cloud;
        }

        static typename pcl::PointCloud<PointT>::Ptr openAndLoadPcdFile(const std::filesystem::path &path) {
            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::io::loadPCDFile(path.string(), *cloud);
            std::cout << "Loaded a Pcd file with [" << cloud->size() << "] point(s)" << std::endl;
            return cloud;
        }

        static typename pcl::PointCloud<PointT>::Ptr
        openAndLoadCloudFile(const std::filesystem::path &file) noexcept(false) {
            typename pcl::PointCloud<PointT>::Ptr cloud{};
            const auto extension{file.extension()};
            if (extension == PLY_EXTENSION)
                return openAndLoadPlyFile(file);
            else if (extension == PCD_EXTENSION)
                return openAndLoadPcdFile(file);
            throw std::runtime_error("Unsupported cloud file extension.");
        }

        template<std::ranges::input_range Range>
        static std::vector<typename pcl::PointCloud<PointT>::Ptr>
        openAndLoadCloudFromPaths(Range &&files) noexcept(false) {
            std::vector<typename pcl::PointCloud<PointT>::Ptr> clouds{};
            std::ranges::for_each(files, [&clouds](const auto &file) {
                clouds.push_back(openAndLoadCloudFile(file));
            });
            return clouds;
        }

    public:

        template<std::ranges::input_range Range>
        explicit Loader(const Range &range) :
                safeIndex(std::make_unique<SafeIndexType>(0, std::ranges::size(range))) {
            std::ranges::copy(range, std::back_inserter(files));
        }

        virtual ~Loader() = default;

    protected:
        std::vector<std::filesystem::path> files{};
        std::unique_ptr<SafeIndexType> safeIndex{nullptr};
    };

}