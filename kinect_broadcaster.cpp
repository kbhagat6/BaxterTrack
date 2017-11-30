#include <ros/ros.h>
#include <tf/transform_broadcaster.h>



int main(int argc, char** argv){
	ros::init(argc, argv, "kinect_broadcaster");
	ros::NodeHandle n;
	
	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	while(n.ok()){
  	     broadcaster.sendTransform(
	     tf::StampedTransform(
	     tf::Transform(tf::Pose(tf::Quaternion(-0.415, 0.443, 0.586, 0.537), tf::Vector3(1.115, -0.182, 0.609))),
	     //tf::Transform(tf::Quaternion(-0.415, 0.443, 0.586, 0.537), tf::Vector3(1.115, -0.182, 0.609)),
	     ros::Time::now(),"torso","camera_link"));
             /*
	     tf::Transform(tf::Quaternion(0.414, -0.438, -0.595, 0.531), tf::Vector3(0.733, 1.069, 0.050)),
	     ros::Time::now(),"camera_link","torso"));*/
	     r.sleep();
        }

}





