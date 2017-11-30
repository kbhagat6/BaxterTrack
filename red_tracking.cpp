#include <ros/ros.h>
#include <iostream>
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>   //converts ros image message to opencv messages
 #include <opencv2/features2d/features2d.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/registration.h>
#include <tf/transform_listener.h>
#include <pcl/recognition/linemod.h>


//pcl::PointCloud< PointT >::operator()(int u, int v)

 using namespace cv; 
 using namespace cv_bridge;

 static const std::string OPENCV_WINDOW = "Image window";

 class RedTracking
 { 
   ros::NodeHandle nh_;  //starts the node 
   image_transport::ImageTransport it_;		// forpublishing and subscribing to images in ROS allows you to subscribe to compressed image streams
   image_transport::Subscriber image_sub_;
   tf::TransformListener listener;
  // image_transport::Subscriber depth_sub_;
   image_transport::Publisher image_pub_;
   
   ros::Publisher finalpts_pub;
   ros::Subscriber point_cloud_sub_;
   Mat depth;
   int xcoord;
   int ycoord;
 //  image_transport::Publisher depth_pub_;
   // Mat depth;
	

 public:
   RedTracking()
     : it_(nh_)
   {
     // Subscribe to input video feed and publish output video feed
     image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
       &RedTracking::imageCb, this);
     
    //depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &RedTracking::depthCb, this);
    point_cloud_sub_= nh_.subscribe("/camera/depth_registered/points",1,&RedTracking::pclfunc, this);
     

    finalpts_pub = nh_.advertise<geometry_msgs::PointStamped>("/finalpts", 100);
     image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
     cv::namedWindow(OPENCV_WINDOW);

   } 
 
   ~RedTracking()
   {
     destroyWindow(OPENCV_WINDOW);
   }
 
   void imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
     cv_bridge::CvImagePtr cv_ptr;

     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // first convert the ROS image message to a CvImage suitable for working with OpenCV. Since we're going to draw on the image, we need a mutable copy of it, so we use toCvCopy(). 
     }
     catch (cv_bridge::Exception& e) 
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
  // Draw an example circle on the video stream
     // std::cout << cv_ptr->image.rows << "\n";
     //std::cout << cv_ptr->image.cols << "\n";
    /* int test;
     for(int i=0; i < cv_ptr->image.rows; i++){
	
	for(int j=0; j < cv_ptr->image.cols; j++){
	    
	    for(int k=0; k < cv_ptr->image.channels(); k++){
                //Invert the image by subtracting image data from 255               
           	 cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k]; //= 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
		
            }

	}
     }
     
    test= cv_ptr->image.data[300*cv_ptr->image.rows*4+200*3 + 0];
     std::cout << test << "\n";  */     
 Mat image = cv_ptr->image;
Mat greyMat;
// Read image
cv::cvtColor(image, greyMat, CV_BGR2GRAY);

SimpleBlobDetector::Params params;
params.minDistBetweenBlobs = 50;  
params.filterByArea = true;  
params.filterByArea = true;
params.minArea = 1500;
params.blobColor= 255;       
           
params.maxArea = 20000;          
// Set up the detector with default parameters.
SimpleBlobDetector detector;
 
// Detect blobs.
std::vector<KeyPoint> keypoints;
detector.detect( greyMat, keypoints);

 
// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
Mat im_with_keypoints;
drawKeypoints( greyMat, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

for(int i=0; i<keypoints.size(); i++){
   xcoord= keypoints[i].pt.x;
   ycoord=keypoints[i].pt.y;
   std::cout<< keypoints[i].pt.x << "," << keypoints[i].pt.y << "\n";
    //std::cout << cloud(keypoints[i].pt.x, keypoints[i].pt.y).x << "\n";
   // std::cout << cloud(keypoints[i].pt.x, keypoints[i].pt.y).y << "\n";
  //  std::cout << cloud(keypoints[i].pt.x, keypoints[i].pt.y).z << "\n";
}


// Show blobs
imshow("keypoints", im_with_keypoints );


/*x = -0.265176
y = -0.198605
z = 1.165
*/

/*x = -0.264266
y = 0.133791
z = 1.161
*/
      /*
     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
       circle(cv_ptr->image, cv::Point(270, 200), 10, CV_RGB(255,0,0));  */
     
     // Update GUI Window (300, 200
     imshow(OPENCV_WINDOW, cv_ptr->image);
     waitKey(3);
     
     // Output modified video stream
     image_pub_.publish(cv_ptr->toImageMsg());
}


void pclfunc(const sensor_msgs::PointCloud2 cloud){

        
        pcl::PointCloud<pcl::PointXYZ> point_pcl;
	pcl::fromROSMsg(cloud,point_pcl);
	int pcl_index, rgb_h, rgb_w;
         geometry_msgs::PointStamped kinect_point;
  	 kinect_point.header.frame_id = "camera_link";
 
   kinect_point.header.stamp = ros::Time();
        
     if((xcoord >= 0 && ycoord >= 0)&& (xcoord<640 && ycoord <480)){
		rgb_h = ycoord;  //j
		rgb_w =xcoord;   //i
	      // std::cout << xcoord << "," << ycoord << std::endl;
	      //  std::cout << cloud.width<< std::endl;
		pcl_index = ( (rgb_h*cloud.width) + rgb_w); // 
	     //   std::cout << rgb_w <<"," << cloud.height << "," << rgb_h << std::endl;
	         std::cout << pcl_index << std::endl;
	      //  std::cout << point_pcl.size() << std::endl;    //307200
		//std::cout << "(x,y,z) = " << point_pcl.at(pcl_index) << std::endl;
		 /*std::cout << "x = " << point_pcl.at(pcl_index).x << std::endl;
		 std::cout << "y = " << point_pcl.at(pcl_index).y << std::endl;
 		 std::cout << "z = " << point_pcl.at(pcl_index).z << std::endl;
                 std::cout << "\n"; */
             
	     /*kinect_point.point.x=0.778;	//z value	
	     kinect_point.point.y=0-(-0.3143);   // 0-x value
	     kinect_point.point.z=0-(-0.08156);	// 0-y value*/
             
    	     kinect_point.point.x=point_pcl.at(pcl_index).z;
	     kinect_point.point.y=0-point_pcl.at(pcl_index).x;
	     kinect_point.point.z=0-point_pcl.at(pcl_index).y;
	      try{
		     geometry_msgs::PointStamped torso_point;		//for time stamped.
		     listener.transformPoint("torso",kinect_point, torso_point);
		 
		     ROS_INFO("camera_link: (%.3f, %.3f, %.3f) -----> torso: (%.3f, %.3f, %.3f) at time %.2f",
			 (kinect_point.point.x), (kinect_point.point.y), (kinect_point.point.z),
			 torso_point.point.x, torso_point.point.y, torso_point.point.z, torso_point.header.stamp.toSec());
                      
	   	     float kx= 39.3701*kinect_point.point.x;
		     float ky= 39.3701*kinect_point.point.y;
 		     float kz= 39.3701*kinect_point.point.z;
		     
		     float wx= 39.3701*torso_point.point.x;
		     float wy= 39.3701*torso_point.point.y;
 		     float wz= 39.3701*torso_point.point.z;

		     ROS_INFO("camera_link inches: (%.3f, %.3f, %.3f) -----> torso: (%.3f, %.3f, %.3f)",
			 (kx), (ky), (kz), wx, wy, wz);
                    
		
		     finalpts_pub.publish(torso_point);   

		   

	      }
	      catch(tf::TransformException& ex){
	    	 ROS_ERROR("Received an exception trying to transform a point from \"kinect_link\" to \"torso\": %s", ex.what());
	      }
      }


      
  
//	pub.publish(cloud);

}



/*void depthCb(const sensor_msgs::ImageConstPtr& msg) 
  {
    //covert the rosimage into an openCV Mat image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //store the new Mat image inside of a Mat object for easy access

     Mat image = cv_ptr->image;
    uint16_t depth = cv_ptr->image.at<uint16_t>(cv::Point(xcoord,ycoord));
   
  // depth= (float)depth*0.001;
    ROS_INFO("Depth: %d", depth);
    
   // processDepth();
  }*/

 };
 
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "red_tracking");
   RedTracking ic;


 /*  tf::TransformListener listener(ros::Duration(10));
 
   // transform a point once every second
   ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));*/
 
   ros::spin();
   return 0;
 }
