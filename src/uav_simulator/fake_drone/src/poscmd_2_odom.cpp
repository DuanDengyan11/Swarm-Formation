#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber cmd_sub0, cmd_sub1, cmd_sub2, cmd_sub3, cmd_sub4;
ros::Publisher  odom_pub0, odom_pub1, odom_pub2, odom_pub3, odom_pub4;

quadrotor_msgs::PositionCommand cmd0, cmd1, cmd2, cmd3, cmd4;
double _init_x, _init_y, _init_z;
double _init_x1, _init_y1, _init_z1;
double _init_x2, _init_y2, _init_z2;
double _init_x3, _init_y3, _init_z3;
double _init_x4, _init_y4, _init_z4;

bool rcv_cmd0 = false, rcv_cmd1 = false, rcv_cmd2 = false, rcv_cmd3 = false, rcv_cmd4 = false;
// fstream file
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd0 = true;
	cmd0    = cmd;
}

void rcvPosCmdCallBack1(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd1 = true;
	cmd1    = cmd;
}

void rcvPosCmdCallBack2(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd2 = true;
	cmd2    = cmd;
}

void rcvPosCmdCallBack3(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd3 = true;
	cmd3    = cmd;
}

void rcvPosCmdCallBack4(const quadrotor_msgs::PositionCommand cmd)
{	
	rcv_cmd4 = true;
	cmd4    = cmd;
}

void pubOdom(quadrotor_msgs::PositionCommand _cmd, ros::Publisher  _odom_pub, bool rcv_cmd, double initx, double inity, double initz)
{	
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";

	if(rcv_cmd)
	{
	    odom.pose.pose.position.x = _cmd.position.x;
	    odom.pose.pose.position.y = _cmd.position.y;
	    odom.pose.pose.position.z = _cmd.position.z;

		Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
		Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
		Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
		Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
		Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
		Eigen::Vector3d zB = xB.cross(yB);
		Eigen::Matrix3d R;
		R.col(0) = xB;
		R.col(1) = yB;
		R.col(2) = zB;
		Eigen::Quaterniond q(R);  //居然还能生成四元数
	    odom.pose.pose.orientation.w = q.w();
	    odom.pose.pose.orientation.x = q.x();
	    odom.pose.pose.orientation.y = q.y();
	    odom.pose.pose.orientation.z = q.z();

	    odom.twist.twist.linear.x = _cmd.velocity.x;
	    odom.twist.twist.linear.y = _cmd.velocity.y;
	    odom.twist.twist.linear.z = _cmd.velocity.z;

	    odom.twist.twist.angular.x = _cmd.acceleration.x;
	    odom.twist.twist.angular.y = _cmd.acceleration.y;
	    odom.twist.twist.angular.z = _cmd.acceleration.z;
	}
	else
	{
		odom.pose.pose.position.x = initx;
	    odom.pose.pose.position.y = inity;
	    odom.pose.pose.position.z = initz;

	    odom.pose.pose.orientation.w = 1;
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = 0;

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;
	}

    _odom_pub.publish(odom);
}

int main (int argc, char** argv) 
{   
	//由command发送来的数据进行处理
	//相比command多了四元数，通过 _odom_pub 发布话题     
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

    nh.param("init_x", _init_x,  0.0);
    nh.param("init_y", _init_y,  0.0);
    nh.param("init_z", _init_z,  0.0);
	// heli1
	nh.param("init_x1", _init_x1,  0.0);
    nh.param("init_y1", _init_y1,  0.0);
    nh.param("init_z1", _init_z1,  0.0);
	// heli2
	nh.param("init_x2", _init_x2,  0.0);
    nh.param("init_y2", _init_y2,  0.0);
    nh.param("init_z2", _init_z2,  0.0);
	// heli3
	nh.param("init_x3", _init_x3,  0.0);
    nh.param("init_y3", _init_y3,  0.0);
    nh.param("init_z3", _init_z3,  0.0);
	// heli4
	nh.param("init_x4", _init_x4,  0.0);
    nh.param("init_y4", _init_y4,  0.0);
    nh.param("init_z4", _init_z4,  0.0);

    cmd_sub0  = nh.subscribe( "command", 1, rcvPosCmdCallBack );
    odom_pub0 = nh.advertise<nav_msgs::Odometry>("odometry", 1); 

	cmd_sub1  = nh.subscribe( "command1", 1, rcvPosCmdCallBack1 );
    odom_pub1 = nh.advertise<nav_msgs::Odometry>("odometry1", 1);

	cmd_sub2  = nh.subscribe( "command2", 1, rcvPosCmdCallBack2 );
    odom_pub2 = nh.advertise<nav_msgs::Odometry>("odometry2", 1); 

    cmd_sub3  = nh.subscribe( "command3", 1, rcvPosCmdCallBack3 );
    odom_pub3 = nh.advertise<nav_msgs::Odometry>("odometry3", 1);	                     

    cmd_sub4  = nh.subscribe( "command4", 1, rcvPosCmdCallBack4 );
    odom_pub4 = nh.advertise<nav_msgs::Odometry>("odometry4", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom(cmd0, odom_pub0, rcv_cmd0, _init_x, _init_y, _init_z);
		pubOdom(cmd1, odom_pub1, rcv_cmd1, _init_x1, _init_y1, _init_z1);                 
		pubOdom(cmd2, odom_pub2, rcv_cmd2, _init_x2, _init_y2, _init_z2);                 
		pubOdom(cmd3, odom_pub3, rcv_cmd3, _init_x3, _init_y3, _init_z3);                 
		pubOdom(cmd4, odom_pub4, rcv_cmd4, _init_x4, _init_y4, _init_z4);                 

        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}