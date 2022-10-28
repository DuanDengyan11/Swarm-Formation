#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/Assignment.h>
#include <cable_load/cable_load.h>

#include <fstream>
#include <iostream>
using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private: 

  enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      SEQUENTIAL_START
    };   
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;

    cable_load cable_load_;

    /* parameters */
    double no_replan_thresh_, replan_thresh_;
    double planning_horizen_, planning_horizen_time_;
    bool flag_realworld_experiment_;
    double replan_trajectory_time_;

     // global goal setting for swarm
    Eigen::Vector3d swarm_central_pos_;
    double Fcable_[50][3];
    double cable_length_;
    double m_load_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    bool flag_relan_astar_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_;
    ros::Subscriber odom_sub_;
    ros::Publisher poly_traj_pub_;
    ros::Publisher reached_pub_, start_pub_;
    ros::Subscriber central_goal;
    // result file and file name
    string result_fn_;
    fstream result_file_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init); // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    // void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &msg);
    void formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void cal_skew_matrix(Eigen::Vector3d x, Eigen::Matrix3d &x_hat);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM();


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif