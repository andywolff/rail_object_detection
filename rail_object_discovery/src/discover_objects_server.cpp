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
#include <pcl/io/pcd_io.h>

#include <rail_pcl_object_segmentation/ExtractObjects.h>
#include <rail_pcl_object_segmentation/ObjectConstraints.h>

#include "rail_object_discovery/DiscoverObjects.h"
#include "rail_object_discovery/UpdateEnvironment.h"

#include "rail_object_discovery/NamedPointCloud2.h"

#include "tf/transform_datatypes.h"


#define TF_BUFFER_DURATION_SECS 10

std::string sensor_topic_name, target_frame_id, extract_objects_service_name, update_environment_service_name;
bool update_cloud_timestamp_if_too_old, should_update_environment;

tf::TransformListener* tf_listener;

sensor_msgs::PointCloud2::ConstPtr pointCloud;

bool received_pointCloud=false;

bool discover_objects_callback(rail_object_discovery::DiscoverObjects::Request &req,
                               rail_object_discovery::DiscoverObjects::Response &res)
{
  //don't try to discover objects if there is no pointCloud
  if (!received_pointCloud) {
    ROS_ERROR("No pointCloud yet received");
    return false;
  }
  if (!pointCloud) {
    ROS_ERROR("pointCloud was null");
    return false;
  }
  ROS_INFO("Beginning discover_objects_callback");

  sensor_msgs::PointCloud2 transformedCloud;

  //Fail if there is no recent point cloud
  ROS_INFO("Checking whether incoming pointcloud is marked as too old");
  ros::Time tfTimestamp = pointCloud->header.stamp;
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
   
  // We are going to transform the pointCloud into a new frame. This is useful if the current frame will move around undesirably.
  // For example, we usually have clouds with respect to the camera frame, so we must transform them to a more global and static frame like odom

  // Wait for transform to become available as necessary
  if (!tf_listener->waitForTransform(pointCloud->header.frame_id, target_frame_id, tfTimestamp,
				     ros::Duration(3.0)))
    {
      ROS_ERROR("Transform from '%s' to '%s' not available within timeout duration", pointCloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }
  
  // Do the transform
  if (!pcl_ros::transformPointCloud(target_frame_id, *pointCloud, transformedCloud, *tf_listener))
    {
      ROS_ERROR("Unable to transform point cloud from frame '%s' to '%s'", pointCloud->header.frame_id.c_str(), target_frame_id.c_str());
      return false;
    }
 
  
  //call extract objects server
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceClient client = n.serviceClient<rail_pcl_object_segmentation::ExtractObjects>(
      extract_objects_service_name);

  rail_pcl_object_segmentation::ExtractObjects extract_srv;

  extract_srv.request.cloud = transformedCloud;
  extract_srv.request.constraints = req.constraints;
  extract_srv.request.plane_slope_tolerance = req.plane_slope_tolerance;

  ROS_INFO("Segmenting image...");
  if (client.call(extract_srv))
  { //successful
    if (should_update_environment)
    {
      //Call environment server
      ROS_INFO("Generating environment update service request...");
      ros::ServiceClient env_client = n.serviceClient<rail_object_discovery::UpdateEnvironment>(
          update_environment_service_name);

      rail_object_discovery::UpdateEnvironment env_srv;
      env_srv.request.static_environment = transformedCloud;
      for (unsigned int i=0; i<extract_srv.response.objects.size(); i++)
      {
        rail_pcl_object_segmentation::DiscoveredObject object = extract_srv.response.objects.at(i);
        env_srv.request.objects.push_back(object.objectCloud);
        env_srv.request.centers.push_back(object.center);
        env_srv.request.colors.push_back(object.color);
        env_srv.request.radii.push_back(object.radius);
      }

      // Copy plane clouds for naming
      for (std::vector<rail_pcl_object_segmentation::DiscoveredPlane>::iterator it =
          extract_srv.response.planes.begin(); it != extract_srv.response.planes.end(); ++it)
      {
        env_srv.request.surfaces.push_back((*it).planeCloud);
      }

      if (env_client.call(env_srv))
      {
        //ROS_INFO("Service call complete.");
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
    }
    else
    {
      // Return clouds w/o names
      for (std::vector<rail_pcl_object_segmentation::DiscoveredObject>::iterator it = extract_srv.response.objects.begin();
          it != extract_srv.response.objects.end(); ++it)
      {
        rail_object_discovery::NamedPointCloud2 namedCloud;
        namedCloud.cloud = (*it).objectCloud; // Copy point cloud
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
  if (!received_pointCloud) ROS_INFO("First pointCloud received, ready to extract objects");
  else ROS_DEBUG("Received pointCloud");
  received_pointCloud=true;
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
  priv.param<bool>("update_environment", should_update_environment, true);
  ROS_INFO_STREAM("Will call update environment service: " << should_update_environment);
  priv.param<std::string>("update_environment_service_name", update_environment_service_name, "update_environment");
  if (should_update_environment)
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

