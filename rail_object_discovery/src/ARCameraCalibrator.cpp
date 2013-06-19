#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool calibrate=false;
tf::Transform transform_gripper2camera;

bool calibration_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  //recalculate gripper->openni_pose transform
  ROS_INFO("Beginning camera pose calibration based on ar tag location");
  calibrate=true;
  return true;
}

bool calibration_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Ending calibration and publishing corrected camera pose relative to gripper");
  calibrate=false;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"ar_camera_calibrator");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  ros::ServiceServer start_service = n.advertiseService("ar_camera_calibration_start", calibration_start);
  ros::ServiceServer stop_service = n.advertiseService("ar_camera_calibration_stop", calibration_stop);

  transform_gripper2camera.setOrigin(tf::Vector3(0.04,0,0));
  transform_gripper2camera.setRotation(tf::createQuaternionFromRPY(0,1.57,0));

  ros::Rate rate(10.0);
  while (n.ok()) {

    if (calibrate)
    {

      //tranform from base_footprint to gripper_palm_link
      tf::Transform transform_marker2camera;
      bool ok = true;
      
            //get transform from ar marker to camera
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker",
  		"/xtion_camera",
  		ros::Time(0), //latest
  		transform);
	tf::Quaternion q;
        q.setRPY(0,0,1.57);
	tf::Transform rot;
	rot.setRotation(q);
	transform_marker2camera=rot*transform;
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: couldn't get grippper orientation: %s",ex.what());
        ok=false;
      }

      if (ok) {
	tf_broadcaster.sendTransform(
		tf::StampedTransform(transform_marker2camera,ros::Time::now(),
		"ar_marker_fixed", "xtion_camera"));
	try {
	  tf::StampedTransform transform;
          tf_listener.lookupTransform(
  		"/gripper_palm_link",
  		"/xtion_camera",
  		ros::Time(0), //latest
  		transform);
	  transform_gripper2camera.setOrigin(transform.getOrigin());
	  transform_gripper2camera.setRotation(transform.getRotation());
	}
        catch (tf::TransformException ex) {
          ROS_ERROR("ARCameraCalibrator.cpp: couldn't get grippper to camera transform: %s",ex.what());
        }
      }
    } //end "if calibrate"


    if (!calibrate) {
      tf_broadcaster.sendTransform(
		tf::StampedTransform(transform_gripper2camera,ros::Time::now(),
		"gripper_palm_link", "xtion_camera"));
    }

    ros::spinOnce();
    rate.sleep();
  }

}

/*
//get transform from ar_marker_fixed to ar_marker
      tf::Transform transform_arDiff;
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker",
  		"/ar_marker_fixed",
  		ros::Time(0), //latest
  		transform);
	transform_arDiff.setOrigin(transform.getOrigin());
	transform_arDiff.setRotation(transform.getRotation());
        ROS_INFO("AR orientation difference: %f", 
		tf::tfDot(
			tf::Vector3(0,0,1),
			tf::quatRotate(transform_arDiff.getRotation(),tf::Vector3(0,0,1))
		)
	);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: couldn't get ar marker error: %s",ex.what());
        ok=false;
      }
*/
