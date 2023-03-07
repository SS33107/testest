#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <fstream>

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;
  
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("tool0_pose", 10);

  tf::StampedTransform transform;
  tf::TransformListener listener;
  geometry_msgs::Pose pub_tool0;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
     try{
        listener.lookupTransform("/base", "/tool0", ros::Time(0), transform);

        pub_tool0.position.x = transform.getOrigin().x();
        pub_tool0.position.y = transform.getOrigin().y();
        pub_tool0.position.z = transform.getOrigin().z();
        pub_tool0.orientation.x = transform.getRotation().x();
        pub_tool0.orientation.y = transform.getRotation().y();
        pub_tool0.orientation.z = transform.getRotation().z();
        pub_tool0.orientation.w = transform.getRotation().w();
        pose_pub.publish(pub_tool0);



      }catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
      }
    ros::spinOnce();
    loop_rate.sleep();
  }  

  // ros::spin();

  return 0;
}