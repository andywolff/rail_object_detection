/*!
 * \file update_environment.cpp
 * \brief Defines a node which implements the UpdateEnvironment service.
 * 
 * Upon calling this node's service, the following actions are taken:
 * 	1) All object points are filtered out of the static environment point cloud.
 *  2) All objects currently recognized by planning_environment are cleared.
 *  3) The planning_environment is informed of each object.
 *  4) The collision map is provided with a filtered static environment point cloud for processing.
 * 
 * \author Paul Malmsten, WPI - pmalmsten@gmail.com
 * \author Tim Jenkel, WPI - timj91@wpi.edu
 * \date Sep 21, 2012
 */

#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/io.h>
#include <pcl_ros/point_cloud.h>

#include "rail_object_discovery/NamedPointCloud2.h"
#include "rail_object_discovery/UpdateEnvironment.h"

ros::Publisher collision_object_publisher;
ros::ServiceClient bounding_box_finder;

bool add_cloud_bounding_box_to_collision_environment(const rail_object_discovery::NamedPointCloud2& namedCloud)
{
  // Find 3D bounding box for object
  ROS_INFO_STREAM(" Calling service to compute bounding box...");
  object_manipulation_msgs::FindClusterBoundingBox2 boundingBoxRequest;
  boundingBoxRequest.request.cluster = namedCloud.cloud;

  if (!bounding_box_finder.call(boundingBoxRequest.request, boundingBoxRequest.response))
  {
    ROS_ERROR("Failed to call cluster bounding box service.");
    return false;
  }
  geometry_msgs::PoseStamped pose_stamped = boundingBoxRequest.response.pose;
  geometry_msgs::Vector3 dimensions = boundingBoxRequest.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box 2 3d client returned an error (0.0 bounding box)");
    return false;
  }

  ROS_INFO(" Bounding box determined, publishing object.");

  // Create add object message
  arm_navigation_msgs::CollisionObject objectMsg;
  objectMsg.id = namedCloud.name;
  objectMsg.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  objectMsg.header.stamp = ros::Time::now();
  objectMsg.header.frame_id = boundingBoxRequest.response.pose.header.frame_id;

  // If the ratio of of the bounding box's dimensions are below the threshold, consider it a cylinder
  double objectSizeRatio = boundingBoxRequest.response.box_dims.x / boundingBoxRequest.response.box_dims.y;
  ROS_INFO_STREAM("Object size ratio: " << objectSizeRatio);

  arm_navigation_msgs::Shape object;
  if(namedCloud.name.find("object_") != std::string::npos
     && objectSizeRatio < 1.3) {
    // Set the collision object shape to a cylinder
    ROS_INFO("Interpreting object as cylinder");
    object.type = arm_navigation_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = 0.25 * (boundingBoxRequest.response.box_dims.x + boundingBoxRequest.response.box_dims.y);
    object.dimensions[1] = boundingBoxRequest.response.box_dims.z;
  }
  else {
    // Set the collision object shape to a box
    object.type = arm_navigation_msgs::Shape::BOX;
    object.dimensions.resize(3);
    object.dimensions[0] = boundingBoxRequest.response.box_dims.x;
    object.dimensions[1] = boundingBoxRequest.response.box_dims.y;
    
    // If it is a surface, reduce the bounding box's height
    if(namedCloud.name.find("surface_") != std::string::npos 
       && boundingBoxRequest.response.box_dims.x > boundingBoxRequest.response.box_dims.z
       && boundingBoxRequest.response.box_dims.y > boundingBoxRequest.response.box_dims.z)
      object.dimensions[2] = 0.005;
    else
      object.dimensions[2] = boundingBoxRequest.response.box_dims.z;
  }
  
  // Add the shape to the message
  objectMsg.shapes.push_back(object);
  objectMsg.poses.push_back(boundingBoxRequest.response.pose.pose);

  // Publish collision object
  collision_object_publisher.publish(objectMsg);

  return true;
}

/*!
 * \brief Handles calls to the 'update environment' service.
 */
bool update_environment_callback(rail_object_discovery::UpdateEnvironment::Request& request,
                                 rail_object_discovery::UpdateEnvironment::Response& response)
{
  // TODO: Filter objects out of original point cloud and publish result for static collision avoidance.
  // The intent is that the environment minus all of the discovered objects would be processed by
  // a package like collider (http://www.ros.org/wiki/collider) such that a static collision message
  // suitable for use by the planning_environment (http://www.ros.org/wiki/planning_environment) would be available.
  // This would allow a robot's arm to dodge static obstacles. Collider seemed to segfault frequently for
  // me; perhaps another package would work better.

  // Clear all collision objects
  ROS_INFO("Clearing all existing objects in planning environment.");
  arm_navigation_msgs::CollisionObject clearAllObjectsMsg;
  clearAllObjectsMsg.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  clearAllObjectsMsg.id = "all";
  clearAllObjectsMsg.header.stamp = ros::Time::now();
  clearAllObjectsMsg.header.frame_id = request.static_environment.header.frame_id;
  collision_object_publisher.publish(clearAllObjectsMsg);
  
  // Name each object and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.objects.size(); i++)
  {
    std::stringstream ss;
    ss << "object_" << i;

    // Return named objects
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.objects.at(i);
    namedCloud.name = ss.str();
    response.objects.push_back(namedCloud);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

    if (!add_cloud_bounding_box_to_collision_environment(namedCloud))
      return false;
  }
  
  // Name each surface and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.surfaces.size(); i++)
  {
    std::stringstream ss;
    ss << "surface_" << i;

    // Return named surfaces
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.surfaces.at(i);
    namedCloud.name = ss.str();
    response.surfaces.push_back(namedCloud);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

    if (!add_cloud_bounding_box_to_collision_environment(namedCloud))
      return false;
  }

  ROS_INFO("Service call complete.");
  return true;
}

/*!
 * \brief Main function for update environment service.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "update_environment_server");
  ros::NodeHandle priv("~");
  ros::NodeHandle n;

  // Read parameters
  std::string serviceName, boundingBoxServiceName, staticEnvironmentTopicName, collisionObjectTopicName;

  priv.param<std::string>("service_name", serviceName, "update_environment");
  priv.param<std::string>("bounding_box_service_name", boundingBoxServiceName, "find_cluster_bounding_box2");
  priv.param<std::string>("output/collision_object_topic_name", collisionObjectTopicName, "collision_object");

  // Configure services and topics
  bounding_box_finder = n.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>(boundingBoxServiceName);
  ROS_INFO("Waiting for existence of bounding box service of name '%s'...", boundingBoxServiceName.c_str());
  bounding_box_finder.waitForExistence();

  collision_object_publisher = n.advertise<arm_navigation_msgs::CollisionObject>(collisionObjectTopicName, 1000);
  ROS_INFO("Ready to publish collision objects at name '%s'", collisionObjectTopicName.c_str());

  ros::ServiceServer service = n.advertiseService(serviceName, update_environment_callback);
  ROS_INFO("Update environment service available at name '%s'", serviceName.c_str());

  ros::spin();
  return 0;
}
