#include <iostream>
#include <string>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Range.h"
#include "visualization_msgs/Marker.h"
#include "armadillo"
#include "pose_utils.h"
#include "quadrotor_msgs/PositionCommand.h"

using namespace arma;
using namespace std;

static  string mesh_resource_load, mesh_resource_heli1, mesh_resource_heli2;
static  string mesh_resource_heli3, mesh_resource_heli4;
static double color_r, color_g, color_b, color_a;
static double scale, scale1, scale2, scale3, scale4;

double cross_yaw, cross_pitch, cross_roll, cross_x, cross_y, cross_z;
double cross_yaw1, cross_pitch1, cross_roll1, cross_x1, cross_y1, cross_z1;
double cross_yaw2, cross_pitch2, cross_roll2, cross_x2, cross_y2, cross_z2;
double cross_yaw3, cross_pitch3, cross_roll3, cross_x3, cross_y3, cross_z3;
double cross_yaw4, cross_pitch4, cross_roll4, cross_x4, cross_y4, cross_z4;

bool   cross_config = false;
bool   origin       = false;
bool   isOriginSet  = false;
colvec poseOrigin(6);

ros::Publisher pathPub;
ros::Publisher meshPub;

ros::Publisher pathPub1;
ros::Publisher meshPub1;

ros::Publisher pathPub2;
ros::Publisher meshPub2;

ros::Publisher pathPub3;
ros::Publisher meshPub3;

ros::Publisher pathPub4;
ros::Publisher meshPub4;

tf::TransformBroadcaster* broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path             pathROS;
visualization_msgs::Marker meshROS;
string _frame_id;
int _drone_id;

void odom_callback1(const nav_msgs::Odometry::ConstPtr& msg)
{
   if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub1.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x1;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y1;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z1;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw1*PI/180.0;
    ypr(1)    += cross_pitch1*PI/180.0;
    ypr(2)    += cross_roll1*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale1;
  meshROS.scale.y = scale1;
  meshROS.scale.z = scale1;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_heli1;
  meshPub1.publish(meshROS);

}

void odom_callback2(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub2.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x2;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y2;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z2;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw2*PI/180.0;
    ypr(1)    += cross_pitch2*PI/180.0;
    ypr(2)    += cross_roll2*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale2;
  meshROS.scale.y = scale2;
  meshROS.scale.z = scale2;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_heli2;
  meshPub2.publish(meshROS);

}

void odom_callback3(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub3.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x3;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y3;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z3;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw3*PI/180.0;
    ypr(1)    += cross_pitch3*PI/180.0;
    ypr(2)    += cross_roll3*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale3;
  meshROS.scale.y = scale3;
  meshROS.scale.z = scale3;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_heli3;
  meshPub3.publish(meshROS);

}

void odom_callback4(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub4.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x4;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y4;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z4;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw4*PI/180.0;
    ypr(1)    += cross_pitch4*PI/180.0;
    ypr(2)    += cross_roll4*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale4;
  meshROS.scale.y = scale4;
  meshROS.scale.z = scale4;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_heli4;
  meshPub4.publish(meshROS);

}


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (msg->header.frame_id == string("null"))
    return;

  colvec pose(6);  
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);

  q(0)    = msg->pose.pose.orientation.w;
  q(1)    = msg->pose.pose.orientation.x;
  q(2)    = msg->pose.pose.orientation.y;
  q(3)    = msg->pose.pose.orientation.z;
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);

  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;  
  
  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin  = pose;
  }
  if (origin)
  {
    vel  = trans(ypr_to_R(pose.rows(3,5))) * vel;  
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel  = ypr_to_R(pose.rows(3,5)) * vel;  
  }

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3,5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);      

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);
  }

  // Mesh model                                                  
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = msg->pose.pose.position.x + cross_x;
  meshROS.pose.position.y = msg->pose.pose.position.y + cross_y;
  meshROS.pose.position.z = msg->pose.pose.position.z + cross_z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += cross_yaw*PI/180.0;
    ypr(1)    += cross_pitch*PI/180.0;
    ypr(2)    += cross_roll*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource_load;
  meshPub.publish(meshROS);                                                  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle n("~");

  n.param("mesh_resource_load", mesh_resource_load, std::string("package://odom_visualization/meshes/load.stl"));
  n.param("mesh_resource_heli1", mesh_resource_heli1, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli2", mesh_resource_heli2, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli3", mesh_resource_heli3, std::string("package://odom_visualization/meshes/heli.obj"));
  n.param("mesh_resource_heli4", mesh_resource_heli4, std::string("package://odom_visualization/meshes/heli.obj"));

  n.param("color/r", color_r, 1.0);
  n.param("color/g", color_g, 0.0);
  n.param("color/b", color_b, 0.0);
  n.param("color/a", color_a, 1.0);
  n.param("origin", origin, false);  
  n.param("frame_id",   _frame_id, string("world") ); 
  n.param("cross_config", cross_config, false);    

  n.param("robot_scale", scale, 2.0);    
  n.param("cross_yaw", cross_yaw, 0.0);
  n.param("cross_pitch", cross_pitch, 0.0);
  n.param("cross_roll", cross_roll, 0.0);
  n.param("cross_x", cross_x, 0.0);
  n.param("cross_y", cross_y, 0.0);
  n.param("cross_z", cross_z, 0.0);
  
  ros::Subscriber sub_odom = n.subscribe("odom", 100,  odom_callback);
  ros::Subscriber sub_odom1 = n.subscribe("odom1", 100,  odom_callback1);
  ros::Subscriber sub_odom2 = n.subscribe("odom2", 100,  odom_callback2);
  ros::Subscriber sub_odom3 = n.subscribe("odom3", 100,  odom_callback3);
  ros::Subscriber sub_odom4 = n.subscribe("odom4", 100,  odom_callback4);

  //load 
  pathPub   = n.advertise<nav_msgs::Path>(            "path",                100, true);
  meshPub   = n.advertise<visualization_msgs::Marker>("robot",               100, true);  

  //heli1
  meshPub1   = n.advertise<visualization_msgs::Marker>("robot1",               100, true);  
  pathPub1   = n.advertise<nav_msgs::Path>(            "path1",                100, true);
  n.param("robot_scale1", scale1, 2.0);    
  n.param("cross_yaw1", cross_yaw1, 0.0);
  n.param("cross_pitch1", cross_pitch1, 0.0);
  n.param("cross_roll1", cross_roll1, 0.0);
  n.param("cross_x1", cross_x1, 0.0);
  n.param("cross_y1", cross_y1, 0.0);
  n.param("cross_z1", cross_z1, 0.0);

  //heli2
  meshPub2   = n.advertise<visualization_msgs::Marker>("robot2",               100, true);  
  pathPub2   = n.advertise<nav_msgs::Path>(            "path2",                100, true);
  n.param("robot_scale2", scale2, 2.0);    
  n.param("cross_yaw2", cross_yaw2, 0.0);
  n.param("cross_pitch2", cross_pitch2, 0.0);
  n.param("cross_roll2", cross_roll2, 0.0);
  n.param("cross_x2", cross_x2, 0.0);
  n.param("cross_y2", cross_y2, 0.0);
  n.param("cross_z2", cross_z2, 0.0);

  //heli1
  meshPub3   = n.advertise<visualization_msgs::Marker>("robot3",               100, true);  
  pathPub3   = n.advertise<nav_msgs::Path>(            "path3",                100, true);
  n.param("robot_scale3", scale3, 2.0);    
  n.param("cross_yaw3", cross_yaw3, 0.0);
  n.param("cross_pitch3", cross_pitch3, 0.0);
  n.param("cross_roll3", cross_roll3, 0.0);
  n.param("cross_x3", cross_x3, 0.0);
  n.param("cross_y3", cross_y3, 0.0);
  n.param("cross_z3", cross_z3, 0.0);

  //heli1
  meshPub4   = n.advertise<visualization_msgs::Marker>("robot4",               100, true);  
  pathPub4   = n.advertise<nav_msgs::Path>(            "path4",                100, true);
  n.param("robot_scale4", scale4, 2.0);    
  n.param("cross_yaw4", cross_yaw4, 0.0);
  n.param("cross_pitch4", cross_pitch4, 0.0);
  n.param("cross_roll4", cross_roll4, 0.0);
  n.param("cross_x4", cross_x4, 0.0);
  n.param("cross_y4", cross_y4, 0.0);
  n.param("cross_z4", cross_z4, 0.0);

  tf::TransformBroadcaster b;
  broadcaster = &b;

  ros::spin();

  return 0;
}
