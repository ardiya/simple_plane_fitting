#include "types.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

struct ClusteringOutput
{
    bool success;
    std::vector<pcl::PointIndices> clusters;
    cloud_ptr_t colored_cloud;
    ClusteringOutput() : success(false) {}
};

ClusteringOutput run_clustering(cloud_ptr_t cloud_ptr)
{
    // normal estimation
    pcl::search::Search<point_t>::Ptr tree(new pcl::search::KdTree<point_t>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<point_t, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_ptr);
    ne.setRadiusSearch(0.01);
    // ne.setKSearch(12);
    ne.compute(*normal_ptr);

    // region growing segmentation
    pcl::RegionGrowing<point_t, pcl::Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(100000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(8);
    reg.setInputCloud(cloud_ptr);
    reg.setInputNormals(normal_ptr);
    reg.setSmoothnessThreshold(2.5 / 180.0 * M_PI);
    // reg.setCurvatureThreshold(1.0);
    reg.setResidualTestFlag(true);
    reg.setResidualThreshold(0.25);

    ClusteringOutput output;
    reg.extract(output.clusters);
    output.colored_cloud = reg.getColoredCloud();
    output.success = !output.clusters.empty();

    return output;
}
