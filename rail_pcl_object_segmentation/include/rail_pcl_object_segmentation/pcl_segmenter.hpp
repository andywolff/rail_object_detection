#ifndef _PCL_SEGMENTER_H
#define _PCL_SEGMENTER_H

/*!
 * 
 * \file pcl_segmenter.hpp
 * \brief Discovers objects in a point cloud though segmentation and plane extraction.
 * 
 * This code is derived from the PCL example listed here:
 * http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
 * 
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \date Sep 17, 2012
 */

#include <vector>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

#include "rail_pcl_object_segmentation/common.hpp"

namespace rail
{
/**
 * \brief Downsamples a point cloud.
 * 
 * This function downsamples the given input cloud using a voxel
 * grid.
 * 
 * \param [in] inputCloud The cloud to downsample.
 * \param [in] leafSizeMeters The size of a leaf in the voxel grid
 *  in meters, along each axis. For example, 0.01f is 1 cm cubed.
 * \param [out] downsampledCloud The filtered cloud. May not be
 *  the same object as the input cloud.
 */
template<typename PointT>
  void DownsampleCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud, float leafSizeMeters,
                       typename pcl::PointCloud<PointT>::Ptr downsampledCloud)
  {
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(inputCloud);
    vg.setLeafSize(leafSizeMeters, leafSizeMeters, leafSizeMeters);
    vg.filter(*downsampledCloud);
  }

/**
 * \brief Downsamples a point cloud.
 * 
 * This function downsamples the given input cloud using a voxel
 * grid.
 * 
 * Uses a default leaf size of 1 cm.
 * 
 * \param [in] inputCloud The cloud to downsample. 
 * \param [out] downsampledCloud The filtered cloud. May not be
 *  the same object as the input cloud.
 */
template<typename PointT>
  void DownsampleCloud(typename pcl::PointCloud<PointT>::Ptr inputCloud,
                       typename pcl::PointCloud<PointT>::Ptr downsampledCloud)
  {
    DownsampleCloud<PointT>(inputCloud, 0.01f, downsampledCloud);
  }

/**
 * \brief Discovers and extracts planes in a point cloud.
 * 
 * This function performs RANSAC plane segmentation and extracts planes
 * with at least the given number of points. Segmentation
 * terminates when a plane with fewer than the expected number of
 * inliers is discovered.
 * 
 * \param [in] inputCloud The cloud to extract planes from.
 * \param [in] ransacIterations The number of RANSAC iterations to perform.
 * \param [in] distanceThreshold The maximum distance that a point may
 *  be away from a plane to be included in the plane's inliers.
 * \param [in] minimumNumberOfPoints The minimum number of inlier points
 *  a plane must have for it to be extracted.
 * \param [out] filteredCloud The resulting cloud with all planes extracted. May
 *  be the same object as inputCloud.
 * \param [out] planes The vector where each discovered plane will be pushed.
 */
template<typename PointT>
  void ExtractPlanes(typename pcl::PointCloud<PointT>::Ptr inputCloud, unsigned int ransacIterations,
                     double distanceThresholdMeters, unsigned int minimumPlaneNumberOfPoints,
                     typename pcl::PointCloud<PointT>::Ptr filteredCloud, std::vector<DiscoveredPlanePtr>& planes)
  {

    pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
    pcl::ModelCoefficients coefficients;
    typename pcl::PointCloud<PointT> plane_inliers;

    // Temporary filtered cloud; initialied to a copy of inputCloud
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered = typename pcl::PointCloud<PointT>::Ptr(
        new typename pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*inputCloud, *cloud_filtered);
    cloud_filtered->header = inputCloud->header; // Persist header information

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransacIterations);
    seg.setDistanceThreshold(distanceThresholdMeters);

    do
    {
      if (cloud_filtered->size() == 0)
      {
        break;
      }

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, coefficients);
      if (inliers->indices.size() == 0)
      {
        break;
      }

      if (inliers->indices.size() < minimumPlaneNumberOfPoints)
      {
        break;
      }

      // Create new result plane message
      DiscoveredPlanePtr plane(new rail_pcl_object_segmentation::DiscoveredPlane());

      // Extract the planar inliers
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);

      extract.filter(plane_inliers);
      pcl::toROSMsg(plane_inliers, plane->planeCloud);
      plane->a = coefficients.values[0];
      plane->b = coefficients.values[1];
      plane->c = coefficients.values[2];
      plane->d = coefficients.values[3];

      // Save the result message
      planes.push_back(plane);

      // Remove the planar inliers, extract the rest
      typename pcl::PointCloud<PointT>::Ptr cloud_filtered_minus_plane = typename pcl::PointCloud<PointT>::Ptr(
          new typename pcl::PointCloud<PointT>());

      extract.setNegative(true);
      extract.filter(*cloud_filtered_minus_plane);

      cloud_filtered = cloud_filtered_minus_plane;
    } while (inliers->indices.size() > minimumPlaneNumberOfPoints);

    // Copy the filtered temporary cloud to the output
    pcl::copyPointCloud(*cloud_filtered, *filteredCloud);
  }

/**
 * \brief Discovers and extracts planes in a point cloud.
 * 
 * This function performs RANSAC plane segmentation and extracts planes
 * with at least the given number of points. Segmentation
 * terminates when a plane with fewer than the expected number of
 * inliers is discovered.
 * 
 * By default, RANSAC is performed with 300 iterations and a distance
 * tolerance of 0.018 meters. Planes with more than 1000 inliers
 * are extracted.
 * 
 * \param [in] inputCloud The cloud to extract planes from.
 * \param [out] filteredCloud The resulting cloud with all planes extracted. May
 *  be the same object as inputCloud.
 * \param [out] planes The vector where each discovered plane will be pushed.
 */
template<typename PointT>
  void ExtractPlanes(typename pcl::PointCloud<PointT>::Ptr inputCloud,
                     typename pcl::PointCloud<PointT>::Ptr filteredCloud, std::vector<DiscoveredPlanePtr>& planes)
  {
    ExtractPlanes<PointT>(inputCloud, 300, 0.018, 1000, filteredCloud, planes);
  }

/*!
 * \brief Discovers objects in the given point cloud.
 * 
 * This function accepts a point cloud produced by a 3d sensor. 
 * Finally, it performs kd-tree clustering on the remaining point clouds
 * and returns the resulting clusters.
 * 
 * This function performs PCL Euclidian Clustering as in the
 * tutorial available here: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
 * 
 * \param [in] cloud The point cloud to process.
 * \param [in] clusterTolerance The maximum distance that two points may be
 *  from one another to be included in the same object.
 * \param [in] minClusterSize The minimum number of points that a object
 *  cluster must contain to be included in the results.
 * \param [in] maxClusterSize The maximum number of points that may
 *  be included in any given object cluster.
 * \param [out] objectClouds The vector where discovered object clouds will be pushed.
 */
template<typename PointT>
  void ExtractObjectClouds(typename pcl::PointCloud<PointT>::Ptr cloud, double clusterTolerance,
                           unsigned int minClusterSize, unsigned int maxClusterSize,
                           std::vector<typename pcl::PointCloud<PointT>::Ptr>& objectClouds)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_f(new typename pcl::PointCloud<PointT>);

    if (cloud->size() == 0)
      return; // KdTreeSearch::setInputCloud will segfault if too few points are present.

    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<typename pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      // Extract discovered cluster into a point cloud
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Persist header information
      cloud_cluster->header = cloud->header;

      objectClouds.push_back(cloud_cluster);
    }
  }

/*!
 * \brief Discovers objects in the given point cloud.
 * 
 * This function accepts a point cloud produced by a 3d sensor. 
 * Finally, it performs kd-tree clustering on the remaining point clouds
 * and returns the resulting clusters.
 * 
 * This function performs PCL Euclidian Clustering as in the
 * tutorial available here: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
 * 
 * This function defaults to a point tolerance of 4 cm, a minimum
 * size of 25 points in an object, and a maximum of 10,000 points
 * in an object.
 * 
 * \param [in] cloud The point cloud to process.
 * \param [out] objectClouds The vector where discovered object clouds will be pushed.
 */
template<typename PointT>
  void ExtractObjectClouds(typename pcl::PointCloud<PointT>::Ptr cloud,
                           std::vector<typename pcl::PointCloud<PointT>::Ptr>& objectClouds)
  {
    ExtractObjectClouds<PointT>(cloud, 0.04, 20, 10000, objectClouds);
  }
}

#endif
