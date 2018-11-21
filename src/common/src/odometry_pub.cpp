/*
This program is introduce something about the message of nav_msgs/Odometry
it can publish the Odometry message,and the transformation of Tf.We use the 
virtual data,it means that surcribe the Odometry message is not from the wheels'
encoder data,but from the known velocity of mobile robots.
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include"common/Encoder.h"
#include<math.h>
#include<boost/assign/list_of.hpp>

#define ODOM_COMBINE

common::Encoder enc;

void sensorMsg_Callback(const common::Encoder& msg){
    enc.leftEncoder=msg.leftEncoder;
    enc.rightEncoder=msg.rightEncoder;
    enc.vx = msg.vx;
    enc.w = msg.w;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"odometry_pub");
	ros::NodeHandle n;
	
	ros::Subscriber sensor_sub=n.subscribe("encoder",1,sensorMsg_Callback);
	ros::Publisher odom_pub=n.advertise<nav_msgs::Odometry>("odom_raw", 50); //publiser  raw odom
#ifndef ODOM_COMBINE
	tf::TransformBroadcaster odom_broadcaster; // define /odom -> /base_link tf broadcaster
#endif


	// initial position and pose
	double x=0.0;
	double y=0.0;
    double th=0.0;
	// define linear.x and angular.z
	double vx=0;
    double w=0;

	ros::Time current_time,last_time;
	current_time=ros::Time::now();
	last_time=ros::Time::now();
	ros::Rate rate(20);// odom publish frequency

    while(n.ok())
    {
        ros::spinOnce(); //check message
        current_time=ros::Time::now();
        double dt=(current_time-last_time).toSec();

        //ROS_INFO("right left: [%d,%d,%d,%d]",enc.right_enc, enc.left_enc);
	
        vx = (float)enc.vx/1000.0;
        w =  (float)enc.w/1000.0;
        x += vx*cos(th)*dt;
        y += vx*sin(th)*dt;
        th += w*dt;
        //ROS_INFO("Yaw angle: %f",th*180.0/M_PI);
        while(th >= M_PI)  th -= 2*M_PI;
        while(th <= -M_PI)  th += 2*M_PI;
        ROS_INFO("Yaw: %f degree",th*180.0/M_PI);

        //get quaterion from yaw, (generated from encoder)
        geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(th);

        // first, publish the transform over tf, do not publish if odom combine needed
#ifndef ODOM_COMBINE
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
#endif

        // next, publish the odom message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // set pose
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.pose.covariance = boost::assign::list_of(0.1) (0) (0) (0) (0) (0)
			                                       (0) (0.1) (0) (0) (0) (0)
												   (0) (0) (1e5) (0) (0) (0)
												   (0) (0) (0) (1e5) (0) (0)
												   (0) (0) (0) (0) (1e5) (0)
												   (0) (0) (0) (0) (0) (0.05);
        // set velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w;
        odom.twist.covariance = boost::assign::list_of(0.001) (0) (0) (0) (0) (0)
			                                      (0) (0.001) (0) (0) (0) (0)
												  (0) (0) (1e5) (0) (0) (0)
												  (0) (0) (0) (1e5) (0) (0)
                                                  (0) (0) (0) (0) (1e5) (0)
												  (0) (0) (0) (0) (0) (0.01);
        odom_pub.publish(odom);
        last_time=current_time; 
        rate.sleep();
    }
    return 0;
} 
