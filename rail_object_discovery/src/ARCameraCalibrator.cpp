#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//count-down timer for calibration. calibration occurs when this is positive
const int counter_max = 20;
int counter = 0;
//maximum amount to linearly interpolate
const double lerp_max = 0.5f;


bool recalibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  //recalculate gripper->openni_pose transform
  ROS_INFO("Beginning camera pose calibration based on ar tag location");
  counter=counter_max;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc,argv,"ar_camera_calibrator");
  ros::NodeHandle n;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  ros::ServiceServer service = n.advertiseService("ar_camera_calibrate", recalibrate);

  //transform from gripper_palm_link to xtion_pose
  tf::Transform xtion_pose;

  ros::Rate rate(10.0);
  while (n.ok()) {

    if (counter>0)
    {
      //transform from ar_marker_fixed to ar_marker
      tf::Transform transform_arDiff;
      bool ok = true;
      //get current gripper2xtion transform
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/gripper_palm_link",
		"/xtion_pose",
  		ros::Time(0), //latest
  		transform);
	xtion_pose.setOrigin(transform.getOrigin());
	xtion_pose.setRotation(transform.getRotation());
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: %s",ex.what());
        ok=false;
      }
      //get arDiff transform
      try {
        tf::StampedTransform transform;
        tf_listener.lookupTransform(
  		"/ar_marker_fixed",
  		"/ar_marker",
  		ros::Time(0), //latest
  		transform);
	transform_arDiff.setOrigin(transform.getOrigin());
	transform_arDiff.setRotation(transform.getRotation());
      }
      catch (tf::TransformException ex) {
        ROS_ERROR("ARCameraCalibrator.cpp: %s",ex.what());
        ok=false;
      }

      if (ok) {
        //shrink the merge value over time to get an average
        double merge = (double)counter/(double)counter_max*lerp_max;
        //update the camera pose
        xtion_pose.setOrigin(xtion_pose.getOrigin()+transform_arDiff.getOrigin()*merge);
        xtion_pose.setRotation(xtion_pose.getRotation().slerp(transform_arDiff.getRotation(),merge));
      }
      counter--;
    } //end "if counter>0"

    tf_broadcaster.sendTransform(
	tf::StampedTransform(xtion_pose,ros::Time::now(),
	"gripper_palm_link", "xtion_pose"));

    ros::spinOnce();
    rate.sleep();
  }

}
