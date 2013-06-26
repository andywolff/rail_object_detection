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

#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/io.h>
#include <pcl_ros/point_cloud.h>

#include "rail_pcl_object_segmentation/pcl_measurement.hpp"

#include "rail_object_discovery/NamedPointCloud2.h"
#include "rail_object_discovery/UpdateEnvironment.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "moveit_msgs/CollisionObject.h"

#define MAX_OBJECTS 20
#define MAX_PLANES 10
ros::Publisher object_pubs[MAX_OBJECTS];
ros::Publisher plane_pubs[MAX_PLANES];
ros::Publisher environment_pub;
ros::Publisher names_pub;
ros::Publisher collision_object_pub;

ros::ServiceClient bounding_box_finder;


//clears all published clouds, names, and objects
void clear() {
  sensor_msgs::PointCloud2 emptyCloud;
  emptyCloud.header.frame_id="/odom";  //TODO: get this from somewhere
  visualization_msgs::MarkerArray markers;
  int i;
  for (i=0; i<MAX_OBJECTS; i++) {

    //publish the empty cloud to clear the object
    object_pubs[i].publish(emptyCloud);

    //add an empty marker to clear the name
    visualization_msgs::Marker marker;
    marker.header.frame_id=emptyCloud.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns="extract_objects";
    marker.id=i;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);

    /*//remove all potential collisionObjects
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id=emptyCloud.header.frame_id;
    std::stringstream ss;
    ss << "object_" << i;
    collision_object.id=ss.str();
    collision_object.operation=collision_object.REMOVE;
    //publish the collision_object
    collision_object_pub.publish(collision_object);*/
  }
  //publish the empty markers to clear the names
  names_pub.publish(markers);

  // Clear all collision objects
//  ROS_INFO("Clearing all existing objects in planning environment.");
  moveit_msgs::CollisionObject clearAllObjectsMsg;
  clearAllObjectsMsg.operation = clearAllObjectsMsg.REMOVE;
  clearAllObjectsMsg.id = "all";
  clearAllObjectsMsg.header.stamp = ros::Time::now();
  collision_object_pub.publish(clearAllObjectsMsg);

  for (i=0; i<MAX_PLANES; i++) {
    //publish the empty cloud to clear the plane
    plane_pubs[i].publish(emptyCloud);
  }
}

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


  // Add the object as a collisionObject to the moveIt planning scene
  // Create add object message
  moveit_msgs::CollisionObject objectMsg;
  objectMsg.id = namedCloud.name;
  objectMsg.operation = objectMsg.ADD;

  objectMsg.header.stamp = ros::Time::now();
  objectMsg.header.frame_id = boundingBoxRequest.response.pose.header.frame_id;

  // Note that x >= y by the way the bounding box is extracted: "x-axis is aligned with the direction of the largest point cloud variance"

  shape_msgs::SolidPrimitive primitive;
  // Set the collision object shape to a box //TODO: consider cylinders or spheres
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = boundingBoxRequest.response.box_dims.x;
  primitive.dimensions[primitive.BOX_Y] = boundingBoxRequest.response.box_dims.y;
  primitive.dimensions[primitive.BOX_Z] = boundingBoxRequest.response.box_dims.z;
  
  // Add the shape to the message
  objectMsg.primitives.push_back(primitive);
  objectMsg.primitive_poses.push_back(boundingBoxRequest.response.pose.pose);

  // Publish collision object
  collision_object_pub.publish(objectMsg);

  return true;
}

/*!
 * \brief Handles calls to the 'update environment' service.
 */
bool update_environment_callback(rail_object_discovery::UpdateEnvironment::Request& request,
                                 rail_object_discovery::UpdateEnvironment::Response& response)
{
  ROS_INFO_STREAM("Updating Environment Service: received " << request.objects.size() << " object clouds, with " << request.centers.size() << " centers, " << request.colors.size() << " colors, " << request.radii.size() << " radii, and " << request.names.size() << " names.");
  // TODO: Filter objects out of original point cloud and publish result for static collision avoidance.
  // The intent is that the environment minus all of the discovered objects would be processed by
  // a package like collider (http://www.ros.org/wiki/collider) such that a static collision message
  // suitable for use by the planning_environment (http://www.ros.org/wiki/planning_environment) would be available.
  // This would allow a robot's arm to dodge static obstacles. Collider seemed to segfault frequently for
  // me; perhaps another package would work better.

  clear();
  
  int combined_clouds = 0;
  sensor_msgs::PointCloud2 floorlessCloud; //a cloud generated by combining objects and non-horizontal planes
  //TODO: ^ subtract floors from the whole cloud instead so limbo pixels don't disappear

  visualization_msgs::MarkerArray markers;

  // Name each object and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.objects.size(); i++)
  {

    //if any of the optional parameters weren't given for this cloud, generate them
    if (i >= request.centers.size() || i >= request.colors.size() || i >= request.radii.size()) {
      ROS_INFO_STREAM("Object " << i << " underdefined.");
      //convert to pcl pointcloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(request.objects.at(i), *pclCloud);
       //if either the center or color needs to be computed, compute them both
       if (i >= request.centers.size() || i >= request.colors.size()) {
         pcl::PointXYZRGB center;
         center = rail::AveragePointCloud(pclCloud);
         //if the center needed to be calculated, add it to the centers
         if (i>=request.centers.size()) {
           ROS_INFO("  Calculated center");
           geometry_msgs::Point ros_center;
           ros_center.x=center.x;
           ros_center.y=center.y;
           ros_center.z=center.z;
           request.centers.push_back(ros_center);
         }
         //if the color needed to be calculated, add it to the colors
         if (i>=request.colors.size()) {
           ROS_INFO("  Calculated color");
           std_msgs::ColorRGBA ros_color;
           ros_color.r=(double)center.r;
           ros_color.g=(double)center.g;
           ros_color.b=(double)center.b;
           ros_color.a=1.0;
           request.colors.push_back(ros_color);
         }
       } //end center or color
       // if the radius needs to be computed, compute it
       if (i >= request.radii.size()) {
         std_msgs::Float64 radius;
         //get the center
         pcl::PointXYZRGB center;
         geometry_msgs::Point ros_center = request.centers.at(i);
         center.x=ros_center.x;
         center.y=ros_center.y;
         center.z=ros_center.z;
         //compute the radius from the center
         radius.data = rail::ComputePointCloudBoundingRadiusFromPoint<pcl::PointXYZRGB>(pclCloud, center);
         request.radii.push_back(radius);
         ROS_INFO("  Calulated radius");
       } //end radius calculation
    } //end center, color, and radius calculation

    std::stringstream ss;
    ss << "object_" << i;

    // Name and publish the object
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.objects.at(i);
    namedCloud.name = ss.str();
    //publish the object cloud to its respective topic
    object_pubs[i].publish(namedCloud.cloud);
    ROS_INFO_STREAM("published an object to /extract_objects/" << ss.str());
    //change the object's name to a user-specified name if the user has specified a name for this cloud
    if (i<request.names.size()) namedCloud.name=request.names.at(i).data;
    response.objects.push_back(namedCloud);

    //add the object to the combined cloud
    if (combined_clouds==0) pcl::copyPointCloud(request.objects.at(i),floorlessCloud);
    else pcl::concatenatePointCloud(floorlessCloud,request.objects.at(i),floorlessCloud);

    //get the object's center
    geometry_msgs::Point center = request.centers.at(i);
    //generate a text marker at the center and color
    visualization_msgs::Marker marker;
    marker.header.frame_id = namedCloud.cloud.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns="extract_objects";
    marker.id=i;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = center.z+request.radii.at(i).data;
    marker.scale.z = 0.1;
    std_msgs::ColorRGBA color = request.colors.at(i);
    marker.color.a = 1.0;
    marker.color.r = 255-color.r;
    marker.color.g = 255-color.g;
    marker.color.b = 255-color.b;
    marker.text=namedCloud.name;
    //add the text marker to the marker array
    markers.markers.push_back(marker);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

    /*if (!*/add_cloud_bounding_box_to_collision_environment(namedCloud);//)
    //  return false;
  }
  names_pub.publish(markers);
  
  // Name each surface and add it to the planning environment
  for (std::vector<sensor_msgs::PointCloud2>::size_type i = 0; i < request.surfaces.size(); i++)
  {
    std::stringstream ss;
    ss << "/extract_objects/plane_" << i;

    // Return named surfaces
    rail_object_discovery::NamedPointCloud2 namedCloud;
    namedCloud.cloud = request.surfaces.at(i);
    namedCloud.name = ss.str();
    response.surfaces.push_back(namedCloud);
    plane_pubs[i].publish(namedCloud.cloud);

    ROS_INFO_STREAM("Adding '" << ss.str() << "' to planning environment:");

      //don't add planes to the planning scene
    //if (!add_cloud_bounding_box_to_collision_environment(namedCloud))
    //  return false;
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

  //generate a publisher for each object and plane
  int i;
  for (i=0; i<MAX_OBJECTS; i++) {
	std::stringstream ss;
        ss << "/extract_objects/object_" << i;
	object_pubs[i]=n.advertise<sensor_msgs::PointCloud2>(ss.str(),1);
  }
  for (i=0; i<MAX_PLANES; i++) {
	std::stringstream ss;
        ss << "/extract_objects/plane_" << i;
	plane_pubs[i]=n.advertise<sensor_msgs::PointCloud2>(ss.str(),1);
  }
  //generate a publisher for the floorless cloud
  environment_pub=n.advertise<sensor_msgs::PointCloud2>("/extract_objects/floorless_cloud",1);

  //generate a publisher for the name text markers
  names_pub = n.advertise<visualization_msgs::MarkerArray>("/extract_objects/names",10);

  //generate a publisher for moveIt collision objects
  collision_object_pub=n.advertise<moveit_msgs::CollisionObject>("/collision_object",10);
  ROS_INFO("Ready to publish collision objects at name '%s'", collisionObjectTopicName.c_str());

  ros::ServiceServer service = n.advertiseService(serviceName, update_environment_callback);
  ROS_INFO("Update environment service available at name '%s'", serviceName.c_str());

  ros::spin();
  return 0;
}
