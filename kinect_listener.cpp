#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
 
 void transformPoint(const tf::TransformListener& listener){   //takes a point in the kinect frame to transform to the world frame
   								//callback for the ros::Timer which will do it every second.
   geometry_msgs::PointStamped kinect_point;
   kinect_point.header.frame_id = "camera_link";
 
   kinect_point.header.stamp = ros::Time();
 
   
   kinect_point.point.x = 1.0;
   kinect_point.point.y = 0.2;
   kinect_point.point.z = 0.0;
 
   try{
     geometry_msgs::PointStamped world_point;		//for time stamped.
     listener.transformPoint("world",kinect_point, world_point);
 
     ROS_INFO("camera_link: (%.6f, %.6f. %.6f) -----> world: (%.6f, %.6f, %.6f) at time %.2f",
         kinect_point.point.x, kinect_point.point.y, kinect_point.point.z,
         world_point.point.x, world_point.point.y, world_point.point.z, world_point.header.stamp.toSec());
   }
   catch(tf::TransformException& ex){
     ROS_ERROR("Received an exception trying to transform a point from \"kinect_link\" to \"world\": %s", ex.what());
   }
 }
 
 int main(int argc, char** argv){
   ros::init(argc, argv, "baxter_tf_listener");
   ros::NodeHandle n;
 
   tf::TransformListener listener(ros::Duration(10));
 
   // transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
 
   ros::spin();
 
 }
