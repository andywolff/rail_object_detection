/*!
 * \file extract_objects_srv.cpp
 * \brief Provides a ROS service for extracting objects and surfaces from a given point cloud.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \date Nov 27, 2012
 */

#include "ros/ros.h"
#include "rail_pcl_object_segmentation/ExtractObjects.h"
#include "rail_pcl_object_segmentation/pcl_measurement.hpp"
#include <vector>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "rail_pcl_object_segmentation/pcl_segmenter.hpp"

struct object_filter : public std::unary_function<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, bool>
{
  double min_distance;
  double max_distance;
  double min_radius;
  double max_radius;

  // Filter function
  bool operator()(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const
  {
    pcl::PointXYZRGB center = rail::AveragePointCloudToPoint<pcl::PointXYZRGB>(cloud);
    double radius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, center);
    ROS_DEBUG("Filter visiting point cloud:");

    if (min_distance >= 0 || max_distance >= 0)
    {
      // Compute center point distance from 0,0
      double distance = sqrt(pow(center.x, 2.0) + pow(center.y, 2.0) + pow(center.z, 2.0));
      ROS_DEBUG_STREAM(" Point cloud distance from origin (m): " << distance);

      if (min_distance >= 0 && distance < min_distance)
      {
        ROS_DEBUG("  Distance too small; discarding object");
        return false;
      }

      if (max_distance >= 0 && distance > max_distance)
      {
        ROS_DEBUG("  Distance too large; discarding object");
        return false;
      }
    }

    ROS_DEBUG_STREAM(" Point cloud spherical radius (m): " << radius);
    if (min_radius >= 0 && radius < min_radius)
    {
      ROS_DEBUG("  Point cloud radius too small; discarding object");
      return false;
    }
    if (max_radius >= 0 && radius > max_radius)
    {
      ROS_DEBUG("  Point cloud radius too large; discarding object");
      return false;
    }

    ROS_DEBUG(" Point cloud OK");
    return true;
  }
};

bool extract(rail_pcl_object_segmentation::ExtractObjects::Request &req,
             rail_pcl_object_segmentation::ExtractObjects::Response &res)
{
  ROS_INFO("Recieved extract object service request");

  //convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(req.cloud, *pclCloud);

  //extract point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());

  rail::DownsampleCloud<pcl::PointXYZRGB>(pclCloud, processed_cloud);

  // Extract planes
  std::vector<rail::DiscoveredPlanePtr> planes;
  rail::ExtractPlanes<pcl::PointXYZRGB>(processed_cloud, processed_cloud, planes);

  // Filter planes which do not appear flat
  if (req.plane_slope_tolerance > 0)
    rail::FilterInclinedPlanes(planes, req.plane_slope_tolerance);

  // Store extracted planes in response
  for (std::vector<rail::DiscoveredPlanePtr>::iterator it = planes.begin(); it != planes.end(); ++it)
  {
    // Pass DiscoveredPlane by value
    res.planes.push_back(*(*it));
  }

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, objects);

  // Filter point clouds
  struct object_filter correct_params;
  correct_params.min_distance = req.constraints.object_min_sensor_range;
  correct_params.max_distance = req.constraints.object_max_sensor_range;
  correct_params.min_radius = req.constraints.object_min_spherical_radius;
  correct_params.max_radius = req.constraints.object_max_spherical_radius;

  ROS_DEBUG("Filtering parameters: ");
  ROS_DEBUG_STREAM(" Object min distance: " << correct_params.min_distance);
  ROS_DEBUG_STREAM(" Object max distance: " << correct_params.max_distance);
  ROS_DEBUG_STREAM(" Object min radius: " << correct_params.min_radius);
  ROS_DEBUG_STREAM(" Object max radius: " << correct_params.max_radius);
  ROS_DEBUG_STREAM(" Plane slope tolerance: " << req.plane_slope_tolerance);

  objects.erase(std::remove_if(objects.begin(), objects.end(), std::not1(correct_params)), objects.end());

  //convert back to point clouds
  sensor_msgs::PointCloud2 object;

  for (int i = 0; i < (int)objects.size(); i++)
  {
    const pcl::PointCloud<pcl::PointXYZRGB> tempCloud = *objects.at(i);
    pcl::toROSMsg(tempCloud, object);
    res.clouds.push_back(object);
  }

  ROS_INFO_STREAM("Returning " << objects.size() << " objects");
  ROS_INFO_STREAM("Returning " << planes.size() << " surface(s)");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_objects_srv");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("extract_objects", extract);
  ROS_INFO("Ready to extract objects");
  ros::spin();

  return 0;
}

