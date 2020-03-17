/*
 * Simple plane fitting with PCL/C++:
 *
 * - Region Growing(normal based clustering)
 * - RANSAC Plane Fitting
 * - Convex Hull to get the outline
 *
 * aditya_a@pretia.co.jp
 *
 * usage: ./simple_plane_fitting [data/0.pcd] [true]
 */

#include "types.hpp"
#include "clustering.hpp"
#include "plane_fitting.hpp"
#include "util.hpp"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main(int argc, char *argv[])
{
    std::string filename = "../data/table.pcd";
    bool visualize = true;
    float downsample_size = 0.00f;
    bool colorless = false;

    if (argc > 1)
    {
        filename = argv[1];
    }

    if (argc > 2 && std::string(argv[2]) == "false")
    {
        visualize = false;
    }
    else
    {
        visualize = true;
    }

    if (argc > 3)
    {
        downsample_size = std::stof(argv[3]);
    }

    if (argc > 4)
    {
        colorless = true;
    }

    std::cout << "Data: " << filename << std::endl
              << "Visualize: " << visualize << std::endl
              << "Downsample: " << downsample_size << std::endl
              << "Colorless: " << colorless << std::endl;

    // load file
    cloud_ptr_t cloud_ptr(new cloud_t);
    pcl::io::loadPCDFile(filename, *cloud_ptr);

    // validate file
    if (cloud_ptr->points.empty())
    {
        std::cout << "Data doesn't exist" << std::endl;
        return -1;
    }

    if (downsample_size > 0.001f)
    {
        downsample(cloud_ptr, downsample_size);
    }

    if (colorless)
    { // turn to red
        for (auto &pt : cloud_ptr->points)
        {
            pt.r = 255;
            pt.g = pt.b = 0;
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
    if (visualize)
    {
        viewer_ptr = boost::make_shared<pcl::visualization::PCLVisualizer>();
        viewer_ptr->addCoordinateSystem(0.1f);

        viewer_ptr->addPointCloud(cloud_ptr);

        std::cout << "Visualizing the original, press [e] or [q] to continue.." << std::endl;
        viewer_ptr->spin();
    }

    // clustering
    auto clustering_result = run_clustering(cloud_ptr);
    if (!clustering_result.success)
    {
        std::cout << "Clustering failed" << std::endl;
    }
    if (visualize)
    {
        viewer_ptr->updatePointCloud(clustering_result.colored_cloud);

        std::cout << "Visualizing the cluster, press [e] or [q] to continue.." << std::endl;
        viewer_ptr->spin();
    }

    // compute plane
    std::vector<PlaneInfo> plane_infos;
    plane_infos.reserve(clustering_result.clusters.size());
    for (auto cluster : clustering_result.clusters)
    {
        cloud_ptr_t cluster_cloud_ptr(new cloud_t);
        pcl::copyPointCloud(*cloud_ptr, cluster, *cluster_cloud_ptr);

        auto plane_data = run_plane_fitting(cluster_cloud_ptr);

        if (!plane_data.success)
            continue;

        plane_infos.push_back(plane_data);
    }
    if (visualize)
    {
        float r = 0.0, g = 1.0, b = 0.0;
        int line_cnt = 0;
        for (const auto &plane : plane_infos)
        {
            const auto &pts = plane.hull_cloud_ptr->points;
            const auto &sz = pts.size();
            for (int i = 0; i < sz; ++i)
            {
                viewer_ptr->addLine(pts[i], pts[(i + 1) % sz], r, g, b, std::to_string(line_cnt++) + "L");
            }
        }
        if (colorless)
            viewer_ptr->updatePointCloud(clustering_result.colored_cloud);
        else
            viewer_ptr->updatePointCloud(cloud_ptr);

        std::cout << "Visualizing the original & plane, press [e] or [q] to continue.." << std::endl;
        viewer_ptr->spin();
    }

    return 0;
}
