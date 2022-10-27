

#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    result_file_.close();
  }
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
 
    have_target_ = false; //是否接收到目标位置点
    have_odom_ = false;  //是否有里程计数据
    have_local_traj_ = false; // 有没有上次规划好的local轨迹

    /*  fsm param  */
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);  // = 1s
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0); // = 1m
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0); // = 7.5m
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0); // = 3s
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false); // = false
    nh.param("fsm/result_file", result_fn_, string("/home/zuzu/Documents/Benchmark/21-RSS-ego-swarm/2.24/ego/ego_swarm.txt")); // 保存结果文件的地方
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0); // = 0.1s

    have_trigger_ = !flag_realworld_experiment_;  // = true

    nh.param("global_goal/cable_length", cable_length_, 1.0);
    nh.param("global_goal/m_load", m_load_, 1.0);

    for (int i = 0; i < 7; i++)
    {
      nh.param("global_goal/Fcable_" + to_string(i) + "/x", Fcable_[i][1], 2.0);
      nh.param("global_goal/Fcable_" + to_string(i) + "/y", Fcable_[i][2], 45.0);
      nh.param("global_goal/Fcable_" + to_string(i) + "/z", Fcable_[i][3], 0.0);
    } //相对目标点的位置

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this); //主程序 先优化吊挂物轨迹 然后优化上面的绳索

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this); //接收到的odom消息，our_lift中我们设定为吊挂物当前位置

    //要发给odom的 
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);

    //记录优化结果
    result_file_.open(result_fn_, ios::app);

    central_goal = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::formationWaypointCallback, this); //地图上鼠标发来的目标位置

  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    ROS_INFO("have_odom_ %ld, have_target_ %ld", have_odom_, have_target_);

    exec_timer_.stop(); // To avoid blockage

    if(!have_odom_ || !have_target_)
      return;
    
    planFromGlobalTraj(1);  //plan load trajectory from global trajectory

    force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.traj_id = data->traj_id;
    msg.start_time = ros::Time(data->start_time);
    msg.order = 5; // todo, only support order = 5 now.

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
  }
  
  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    bool success = false;
    swarm_central_pos_(0) = msg->pose.position.x;
    swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(2) = 0.5;
  
    end_pt_ = swarm_central_pos_;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),  //初始位置、速度、加速度
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); //当前位置作为起始点，目标点作为终止点，不考虑避障等 生成全局轨迹

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    ROS_INFO("have target success %ld", success);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;   //若生成成功
      have_new_target_ = true;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(bool flag_use_poly_init)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(flag_use_poly_init);

    if (!success)
      return false;

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init)
  {
    //优化主程序
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_);  //start_pt_是当前无人机位置，end_pt_是地图上的鼠标给出的目标位置 planning_horizen_是待规划的距离=7.5m // 从当前位置向前规划7.5m 取其位置和速度作为目标

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;
    double desired_start_time;
    if (have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;  //考虑重新规划轨迹所需的时间
      desired_start_pt =
          planner_manager_->traj_.local_traj.traj.getPos(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_vel =
          planner_manager_->traj_.local_traj.traj.getVel(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_acc =
          planner_manager_->traj_.local_traj.traj.getAcc(desired_start_time - planner_manager_->traj_.local_traj.start_time);
    }
    else
    {
      desired_start_pt = start_pt_;
      desired_start_vel = start_vel_;
      desired_start_acc = start_acc_;
    }
    bool plan_success = planner_manager_->reboundReplanForLoad(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        have_local_traj_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj msg;
      polyTraj2ROSMsg(msg);
      poly_traj_pub_.publish(msg); //若轨迹规划成功则发布消息
      have_local_traj_ = true;
    }

    return plan_success;
  }
} // namespace ego_planner
