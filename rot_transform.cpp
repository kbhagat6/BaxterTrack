#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Dense>
 
 int main(int argc, char** argv){
   ros::init(argc, argv, "basismat_transform");
   ros::NodeHandle node;

   tf::TransformListener *tflistener;

    //  kinect_point.header.stamp = ros::Time();
  // right_hand_point.stamp = ros::Time();
   //tf::StampedTransform k_transform;

    ros::Rate rate(10.0);
     tflistener = new tf::TransformListener(node);
       while (node.ok()){
	   try{
	      tf::StampedTransform hand_transform;
	      

	      // listener.waitForTransform("/torso", "/right_hand_camera", ros::Time(0), ros::Duration(4.0) );
	     // listener.lookupTransform("/ar_marker_8", "/camera_link",ros::Time(0), k_transform);
            //  listener.lookupTransform("/world", "/right_hand_camera",ros::Time(0), hand_transform);
	     ros::Time now = ros::Time(0);
 	     tflistener->waitForTransform("/torso", "/camera_link", now, ros::Duration(1.0));
   	     tflistener->lookupTransform("/torso", "/camera_link", now, hand_transform);
	     
  	     tf::Matrix3x3 basismat = hand_transform.getBasis();
	     tf::Quaternion quat = hand_transform.getRotation();
  	     tf::Vector3 trans = hand_transform.getOrigin();

	     double roll, pitch, yaw;
	     
	     //btQuaternion btquat(quat[0],quat[1],quat[2],quat[3]);
             tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	     //std::cout << "RPY:" << roll <<"," << pitch << "," << yaw << std::endl;
	   
	     
	     //std::cout << "trans: "<< trans[0] << "," << trans[1] << "," << trans[2] << std::endl;
	
	    // std::cout << "quart: "<< quat[0] << "," << quat[1] << "," << quat[2] << quat[3] << std::endl << "\n";
	     
	     //std::cout << "basismat:  " << "[" << basismat[0][0] << "," << basismat[0][1] << "," << basismat[0][2] << "\n" << basismat[1][0] << "," << basismat[1][1] << "," << basismat[1][2] << "\n" << basismat[2][0] << "," << basismat[2][1] << "," << basismat[2][2] << "]" << "\n" << "\n";
	     	    
	    
	     Eigen::Matrix4f newmat = Eigen::MatrixXf::Identity(4,4);
 	     Eigen::Matrix3f bmat;
	     bmat <<  basismat[0][0], basismat[0][1], basismat[0][2],basismat[1][0], basismat[1][1], basismat[1][2],basismat[2][0], basismat[2][1], basismat[2][2];
	     newmat.block<3,3>(0,0) = bmat;
             newmat.col(3) = Eigen::Vector4f(trans[0],trans[1],trans[2],1);
             
	     //std::cout << newmat << std::endl;
	     std::cout << "\n";
             
	     Eigen::Vector4f kin_torso(4,1);
             kin_torso(0,0)=0.7;
	     kin_torso(1,0)= 0.5; 
	     kin_torso(2,0)= 0.25;
	     kin_torso(3,0)=1;
	     
	     //std::cout << kin_torso << std::endl;
	     //Eigen::MatrixXd result;
             
	     std::cout << newmat*kin_torso;
 
             
	     //std::cout << newmat.inverse() << std::endl;
	      std::cout << "\n";
	     
             
	     /*geometry_msgs::PointStamped world_point;		//for time stamped.
	     listener.transformPoint("world",kinect_point, world_point);
	     
	     ROS_INFO("camera_link: (%.3f, %.3f. %.3f) -----> ar_marker_8: (%.2f, %.2f, %.2f) at time %.2f",
		 kinect_point.point.x, kinect_point.point.y, kinect_point.point.z,
		 world_point.point.x, world_point.point.y, world_point.point.z, world_point.header.stamp.toSec());
	     */
	   }
	   catch(tf::TransformException& ex){
	       ROS_ERROR("%s",ex.what());
               ros::Duration(1.0).sleep();
  	       continue;
	   }
	   
           ros::Duration(2).sleep();
      }	 
   
 
 
   // transform a point once every second
  // ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
   return 0; 
   //ros::spin();
 
 }
