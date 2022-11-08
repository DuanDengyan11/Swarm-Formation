#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <traj_utils/Assignment.h>
#include "poly_traj_utils.hpp"
#include "munkres_algorithm.hpp"
#include <fstream>
#include <cable_load/cable_load.h>

namespace ego_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    
    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {
  
  private:
    GridMap::Ptr grid_map_;

    cable_load cable_load_;
    
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    ConstrainPoints cps_;

    int drone_id_;
    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_;     // optimization variables
    int piece_num_;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in swarm

    std::vector<Eigen::Vector3d> points_positions;
    Eigen::Matrix<double, 6, 1> FM;

    string result_fn_;
    fstream result_file_;

    double collision_check_time_end_ = 0.0;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_swarm_;                       // swarm weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double weight_cable_length_;
    double weight_cable_colli_;
    double cable_tolerance_;
    double load_dist_;

    double obs_clearance_;                   // safe distance between uav and obstacles
    double swarm_clearance_;                 // safe distance between uav and uav
    double max_vel_, max_acc_;               // dynamic limits
    
    int    formation_size_;
    bool   is_other_assigning_ = false;

    double t_now_;

    double cable_length_, load_mass_;
    double uav_obs_clearance_, uav_swarm_clearance_;
    double weight_uav_obs_, weight_uav_swarm_, weight_FM_feasibility_;


  public:

    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map, const cable_load &CLoad);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinJerkOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    double getCollisionCheckTimeEnd() { return collision_check_time_end_; }

    /* main planning API */
    bool OptimizeTrajectory_lbfgs_forLoad(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points);
                  
    bool OptimizeTrajectory_lbfgs_forCable0(Eigen::MatrixXd accs, Eigen::MatrixXd positions, Eigen::VectorXd durations);
    bool OptimizeTrajectory_lbfgs_forCable(Eigen::Vector3d acc, Eigen::Vector3d position, double cable_coef[6]);
                                            
    void astarWithMinTraj( const Eigen::MatrixXd &iniState, 
                           const Eigen::MatrixXd &finState,
                           std::vector<Eigen::Vector3d> &simple_path,
                           Eigen::MatrixXd &ctl_points,
                           poly_traj::MinJerkOpt &frontendMJ);
  
  
  
  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback_forLoad(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionCallback_forCable(void *func_data, const double *x, double *grad, const int n);

    void addFeasibilityForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);
    void addCollisionForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);
    void addSwarmForCable(Eigen::MatrixXd FMeach, Eigen::VectorXd &grad, Eigen::VectorXd &cost);

    static int earlyExitCallback_forCable(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool CableLengthGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp);

    bool obstacleGradCostP(const int i_dp,
                              const Eigen::Vector3d &p,
                              Eigen::Vector3d &gradp,
                              double &costp);            
    
    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);
    
    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);
    
    bool checkCollision(void);

  public:


    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace ego_planner
#endif