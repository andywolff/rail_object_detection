/*!
 * \file discover_objects_server.cpp
 * \brief Defines a node which implements the discover_objects service.
 *
 * Upon calling this node's service, the following actions are taken:
 *  1) A depth point cloud is captured from a Kinect.
 *  2) The point cloud is transformed into the base_link frame.
 *  3) Objects and planes are extracted from the point cloud
 *  4) (Optional) The UpdateEnvironment service is called with the discovered objects and surfaces.
 *  5) The resulting planes and surfaces are returned.
 *
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \author Tim Jenkel, WPI - timj91@wpi.edu
 * \date Jan 26, 2013
 */

#include <sstream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_ros/transforms.h>

#include <rail_pcl_object_segmentation/ExtractObjects.h>
#include <rail_pcl_object_segmentation/ObjectConstraints.h>

#include "rail_object_discovery/DiscoverObjects.h"
#include "rail_object_discovery/UpdateEnvironment.h"

#include "rail_object_discovery/NamedPointCloud2.h"

#define TF_BUFFER_DURATION_SECS 10

std::string sensor_topic_name, target_frame_id, extract_objects_service_name, update_environment_service_name;
bool update_cloud_timestamp_if_too_old, should_update_enviroment;

tf::TransformListener* tf_listener;

sensor_msgs::PointCloud2::ConstPtr pointCloud;

bool discover_objects_callback(rail_object_discovery::DiscoverObjects::Request &req,
                               rail_object_discovery::DiscoverObjects::Response &res)
{
  sensor_msgs::PointCloud2 transformedCloud;
  ros::Time tfTimestamp = pointCloud->header.stamp;

  // Check whether incoming pointcloud is marked as too old
  if ((ros::Time::now() - ros::Duration(TF_BUFFER_DURATION_SECS - 1)) > pointCloud->header.stamp)
    {
      if (update_cloud_timestamp_if_too_old)
	{
	  ROS_WARN(
		   "Cloud timestamp is too far in the past to compute a tf transformation, using latest transform");
	  tfTimestamp = ros::Time(0);
	}
      else
	{
	  ROS_ERROR("Cloud timestamp is too far in the past to compute a tf transformation");
	  return false;
	}
    }
   
  // Wait for transform to become available as necessary
  if (!tf_listener->waitForTransform(pointCloud->header.frame_id, target_frame_id, tfTimestamp,
				     ros::Duration(3.0)))
    {
      ROS_ERROR(
		"Transform from '%s' to '%s' not available within timeout duration", pointCloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }
  
  // Do the transform
  if (!pcl_ros::transformPointCloud(target_frame_id, *pointCloud, transformedCloud, *tf_listener))
    {
      ROS_ERROR(
		"Unable to transform point cloud from frame '%s' to '%s'", pointCloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }
  
  
  //call extract objects server
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<rail_pcl_object_segmentation::ExtractObjects>(
      extract_objects_service_name);

  rail_pcl_object_segmentation::ExtractObjects extract_srv;

  extract_srv.request.cloud = transformedCloud;
  extract_srv.request.constraints = req.constraints;
  extract_srv.request.plane_slope_tolerance = req.plane_slope_tolerance;

  ROS_WARN("Segmenting image...");
  if (client.call(extract_srv))
  { //successful
    if (should_update_enviroment)
    {
/*
      //Call environment server
      ROS_INFO("Updating environment...");
      ros::ServiceClient env_client = n.serviceClient<rail_object_discovery::UpdateEnvironment>(
          update_environment_service_name);

      rail_object_discovery::UpdateEnvironment env_srv;
      env_srv.request.static_environment = transformedCloud;
      env_srv.request.objects = extract_srv.response.clouds;
*/
      //get the number of planes
      int i=0;
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
         i++;
      }
      ROS_ERROR("extracted %d planes", i);
      //initialize a publisher for each plane
/*      ros::Publisher obj_cloud_pub[i];
      rail_object_discovery::NamedPointCloud2 planes[i];
      i=0;
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
        std::stringstream ss;
        ss << "extracted_objects/plane_" << i;
        obj_cloud_pub[i] = n.advertise<sensor_msgs::PointCloud2>(ss.str(), 1);
        planes[i].name=ss.str();
        i++;
      }
      // Copy plane point clouds for naming
      i=0;
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
          obj_cloud_pub[i++].publish((*it).planeCloud);
          planes[i].cloud=(*it).planeCloud;
//        env_srv.request.surfaces.push_back((*it).planeCloud);
      }
*/
      //do the same for objects
      i=0;
      for (std::vector<sensor_msgs::PointCloud2>::iterator it =
          extract_srv.response.clouds.begin(); it != extract_srv.response.clouds.end(); ++it)
      {
         i++;
      }
      

      ROS_ERROR("extracted %d objects", i);

      return true;
/*      if (env_client.call(env_srv))
      {
        ROS_INFO("Service call complete.");
        // Return named clouds
        res.objects = env_srv.response.objects;
        res.surfaces = env_srv.response.surfaces;

        return true;
      }
      else
      {
        ROS_ERROR("Failed to call environment service");
        return false;
      }
*/
    }
    else
    {
      // Return clouds w/o names
      for (std::vector<sensor_msgs::PointCloud2>::iterator it = extract_srv.response.clouds.begin();
          it != extract_srv.response.clouds.end(); ++it)
      {
        rail_object_discovery::NamedPointCloud2 namedCloud;
        namedCloud.cloud = *it; // Copy point cloud
        res.objects.push_back(namedCloud);
      }

      // Return surfaces w/o names
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
        rail_object_discovery::NamedPointCloud2 namedCloud;
        namedCloud.cloud = (*it).planeCloud; // Copy point cloud
        res.surfaces.push_back(namedCloud);
      }

      ROS_DEBUG("Configured not to update environment; update service call skipped.");
      return true;
    }
  }
  else
  {
    ROS_ERROR("Failed to call extract objects service");
    return false;
  }
}

void sensorCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pointCloud = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "discover_objects_server");
  ros::NodeHandle priv("~");
  ros::NodeHandle n;

  // Required parameters
  if (!priv.getParam("sensor_topic", sensor_topic_name))
    ROS_ERROR("Parameter '~sensor_topic' must be set.");
  if (!priv.getParam("target_frame_id", target_frame_id))
    ROS_ERROR("Parameter '~target_frame_id' must be set.");

  // Optional parameters
  std::string serviceName;

  priv.param<std::string>("service_name", serviceName, "discover_objects");
  priv.param<std::string>("extract_objects_service_name", extract_objects_service_name, "extract_objects");
  ROS_INFO_STREAM("Expecting extract objects service at name '" << extract_objects_service_name << "'");
  priv.param<bool>("update_environment", should_update_enviroment, true);
  ROS_INFO_STREAM("Will call update environment service: " << should_update_enviroment);
  priv.param<std::string>("update_environment_service_name", update_environment_service_name, "update_environment");
  if (should_update_enviroment)
    ROS_INFO_STREAM("Expecting update environment service at name '" << update_environment_service_name << "'");
  priv.param<bool>("update_old_cloud_timestamps", update_cloud_timestamp_if_too_old, false);
  ROS_INFO_STREAM("Will update old cloud timestamps: " << update_cloud_timestamp_if_too_old);

  ros::ServiceServer service = n.advertiseService(serviceName, discover_objects_callback);
  tf_listener = new tf::TransformListener(ros::Duration(TF_BUFFER_DURATION_SECS));

  ros::Subscriber sensorSubscriber = n.subscribe(sensor_topic_name, 1, sensorCallback);

  ROS_INFO_STREAM("Ready to discover objects at name '" << serviceName << "'");
  ros::spin();

  return 0;
}

