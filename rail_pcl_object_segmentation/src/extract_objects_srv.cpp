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
#include "geometry_msgs/Twist.h"

class potential_object
{
  public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointXYZRGB center;
  double radius;
};

struct object_filter : public std::unary_function<potential_object, bool>
{
  double min_distance;
  double max_distance;
  double min_radius;
  double max_radius;

  // Filter function
  bool operator()(const potential_object object) const
  {
    ROS_DEBUG_STREAM("Filter visiting object at (" << object.center.x << ", " << object.center.y << ", " << object.center.z << ")");

    if (min_distance >= 0 || max_distance >= 0)
    {
      // Compute center point distance from 0,0
      double distance = sqrt(pow(object.center.x, 2.0) + pow(object.center.y, 2.0) + pow(object.center.z, 2.0));
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

    ROS_DEBUG_STREAM(" Point cloud spherical radius (m): " << object.radius);
    if (min_radius >= 0 && object.radius < min_radius)
    {
      ROS_DEBUG("  Point cloud radius too small; discarding object");
      return false;
    }
    if (max_radius >= 0 && object.radius > max_radius)
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

  //prepare to extract point clouds
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

  // Extract Objects
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_clouds;
  rail::ExtractObjectClouds<pcl::PointXYZRGB>(processed_cloud, object_clouds);
  // fill in extra object data
  std::vector<potential_object> objects;
  for (unsigned int i=0; i<object_clouds.size(); i++) {
    potential_object object;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = object_clouds.at(i);
    object.cloud = cloud;
    object.center = rail::AveragePointCloud(cloud);
    object.radius = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(cloud, object.center);
    objects.push_back(object);
  }

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

  //convert to DiscoveredObject messages  
  for (int i = 0; i < (int)objects.size(); i++)
  {
    rail_pcl_object_segmentation::DiscoveredObject discovered_object;
    //get the current potential_object
    potential_object object;
    object=objects.at(i);

    //convert the point cloud
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud = object.cloud;
    pcl::toROSMsg(*tempCloud, discovered_object.objectCloud);
    //set the center
    discovered_object.center.x=object.center.x;
    discovered_object.center.y=object.center.y;
    discovered_object.center.z=object.center.z;
    //set the color
    discovered_object.color.r=(double)object.center.r;
    discovered_object.color.g=(double)object.center.g;
    discovered_object.color.b=(double)object.center.b;
    discovered_object.color.a=1.0;
    ROS_INFO_STREAM(" CenterXYZRGB: " << object.center.x << "," << object.center.y << "," << object.center.z << "," << (double)object.center.r << "," << (double)object.center.g << "," << (double)object.center.b );
    //set the radius
    discovered_object.radius.data = object.radius;
    //put the discovered object into the response
    res.objects.push_back(discovered_object);
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

