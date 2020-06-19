#include <stdlib.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Transform.h>
 
using namespace std;
 
 
tf::StampedTransform echo_transform;
tf::TransformListener *tf_listener;
tf::StampedTransform min_distance_trans;
std::pair<int, double> min_distance (-1,100.0);

void listener(){
     // use tf_listener to get the transformation from camera_link to tag 0
     min_distance.first = -1;
     min_distance.second = 100.0;
     for (int i = 0; i < 14;i++){
         string parent_id = "camera_link";
         string child_id = "tag_" + std::to_string(i);
	     
         tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(),    ros::Duration(0.7));
         try {
             tf_listener->lookupTransform(parent_id, child_id, ros::Time(0),     echo_transform);
             std::cout << "At time " << std::setprecision(16) << echo_transform. stamp_.toSec() << std::endl;
             cout << "Frame id:" << echo_transform.frame_id_ << ", Child id:"    << echo_transform.child_frame_id_ << endl;
             double yaw, pitch, roll;
             echo_transform.getBasis().getRPY(roll, pitch, yaw);
             tf::Quaternion q = echo_transform.getRotation();
tf::Vector3 v = echo_transform.getOrigin();
             std::cout << "- Translation: [" << v.getX() << ", " << v.getY() <<  ", " << v.getZ() << "]" << std::endl;
             std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " <<   q.getY() << ", " << q.getZ() << ", " << q.getW() << "]" << std::endl;
 
             /*
               	calucluate the distance from tag_0 to other tags
             */
             double d = 0;
             d = sqrt(pow(v.getX(), 2) + pow(v.getY(), 2) + pow(v.getZ(), 2));
             std::cout << "distance: " << d << std::endl;
		 
	     /*
                 find the closet distance from the tag to camera_link
             */
 
             if (d < min_distance.second)
             {
                 min_distance.first = i;
                 min_distance.second = d;
                 min_distance_trans = echo_transform;  // camera -> i
             }

	 }
         catch (tf::TransformException& ex)
         {
             std::cout << "Exception thrown:" << ex.what() << std::endl;
         }
      }
 
     /*
         find the camera position from map_tag_0
     */
     string parent_id = "origin";
     string child_id = "map_tag_" + std::to_string(min_distance.first);
     tf_listener->waitForTransform(child_id, parent_id, ros::Time::now(), ros::  Duration(0.001));
     try {
 
         tf_listener->lookupTransform(parent_id, child_id, ros::Time(0), echo_transform);
         tf::Transform localization_trans;
          /*
              find transformation matrix from echo_transform and min_distance_trans
          */
 
         localization_trans = echo_transform * min_distance_trans.inverse();
 cout << "=================================\n";
         cout << "*get the robot position*\n";
         cout << "rotation:\n";
         tf::Quaternion q = localization_trans.getRotation();
         cout << "[" << q.getX() << ", " << q.getY() << ", "<< q.getZ() << ", " <<  q.getW() << "]" << endl;
         tf::Vector3 v = localization_trans.getOrigin();
         cout << "translation:\n";
         cout << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" <<  endl;
 
         // broadcast transform from origin to camera_link
         static tf::TransformBroadcaster br;
         tf::StampedTransform output_transform = tf::StampedTransform(localization_trans, ros::Time::now(), "origin", "camera_link");
 
         br.sendTransform(output_transform);
         cout << "=================================\n";
      }
     catch (tf::TransformException& ex)
     {
         std::cout << "Exception thrown:" << ex.what() << std::endl;
     }
     return ;
 }

int main(int argc, char** argv){
     ros::init(argc, argv, "apriltag_tf");
     ros::NodeHandle nh; 
     tf_listener = new tf::TransformListener();
 
     while (ros::ok())
     {   
         ros::spinOnce();
         listener();
     }   
     
     return 0;
 }

