#include<iostream>
using namespace std;
#include <vector>
#include <Eigen/Eigen> 
#include <ros/ros.h>

class cable_load
{
private:

    double length_, height_, width_;
    std::vector<Eigen::Vector3d> cable_points;
    Eigen::Matrix<double, 12, 6> G_inv;
    Eigen::MatrixXd G_null_space;

    Eigen::Matrix3d cal_skew_matrix(Eigen::Vector3d x);
    
public:
    cable_load(/* args */);
    ~cable_load();
    void init(ros::NodeHandle &nh);
};

cable_load::cable_load(/* args */)
{
    
}
cable_load::~cable_load()
{
}
