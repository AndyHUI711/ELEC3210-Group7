#include<ros/ros.h>
#include<string>
#include<iostream>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cstdlib>


//reference: https://blog.csdn.net/just_do_it567/article/details/114303218
double ix, iy, px, py;
 

void pos_callback(const geometry_msgs::PoseStampedConstPtr &pos_msg){
    px = pos_msg->pose.position.x;
    py = pos_msg->pose.position.y;
    //ROS_INFO("pos_msg: %f", px);  //fine
    //ROS_INFO("pos_msg: %f", py);
}
 
int main(int argc,char** argv){
 
    ros::init(argc, argv, "show_trajectory");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("point", 1, true);
    ros::Subscriber pose_sub = nh.subscribe("/slam_out_pose", 1000, pos_callback);
    
    bool is_start = true;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    nav_msgs::Path path;
   
   
    while(ros::ok()){
        current_time = ros::Time::now();
        ros::spinOnce();
        if(is_start){
            ix = px;
            iy = py;
            is_start = false;
        }
 
        //print path
        
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = px-ix;
        this_pose_stamped.pose.position.y = py-iy;
        //ROS_INFO("position: %f", px-ix);  //fine
        //ROS_INFO("position: %f", py-iy);
        
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 1;
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.header.frame_id = "map";
        //ROS_INFO("ok");
 
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);
        
        
    }
 
    return 0;
}
