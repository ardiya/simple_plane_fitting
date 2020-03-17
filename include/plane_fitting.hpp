#include "types.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

struct PlaneInfo
{
    bool success;
    pcl::ModelCoefficients::Ptr coefficients_ptr;
    pcl::PointIndices::Ptr inliers_ptr;
    cloud_ptr_t hull_cloud_ptr;

    PlaneInfo() : success(false), coefficients_ptr(new pcl::ModelCoefficients), inliers_ptr(new pcl::PointIndices), hull_cloud_ptr(new cloud_t)
    {
    }
};

PlaneInfo run_plane_fitting(cloud_ptr_t cloud_ptr)
{
    PlaneInfo output;

    // Create the segmentation object
    pcl::SACSegmentation<point_t> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Run plane segmentation
    seg.setInputCloud(cloud_ptr);
    seg.segment(*output.inliers_ptr, *output.coefficients_ptr);

    if (output.inliers_ptr->indices.size() < 4)
        return output;

    // project point cloud to plane to the plane
    cloud_ptr_t cloud_projected_ptr(new cloud_t);
    pcl::ProjectInliers<point_t> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_ptr);
    proj.setIndices(output.inliers_ptr);
    proj.setModelCoefficients(output.coefficients_ptr);
    proj.filter(*cloud_projected_ptr);

    // Create a Convex Hull representation of the projected inliers
    pcl::ConvexHull<point_t> chull;
    chull.setInputCloud(cloud_projected_ptr);
    chull.reconstruct(*output.hull_cloud_ptr);

    output.success = true;

    return output;
}
