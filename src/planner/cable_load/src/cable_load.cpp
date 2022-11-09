#include <cable_load/cable_load.h>

void cable_load::init(ros::NodeHandle &nh)
{  
    node_ = nh;
    node_.param("optimization/load_length", length_ , 1.0);  //吊挂物长宽高
    node_.param("optimization/load_width", width_ , 1.0);
    node_.param("optimization/load_height", height_ , 1.0);

    double half_length = length_/2.0, half_width = width_/2.0, half_height = height_/2.0;

    cable_points.push_back({-half_length, -half_width, -half_height});
    cable_points.push_back({-half_length, half_width, -half_height});
    cable_points.push_back({half_length, half_width, -half_height});
    cable_points.push_back({half_length, -half_width, -half_height});

    Eigen::Matrix<double, 6, 12> G;
    for (size_t i = 0; i < cable_points.size(); i++)
    {
        G.block<3,3>(0,3*i) << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
        G.block<3,3>(3,3*i) = cal_skew_matrix(cable_points[i]);
    }
    G_inv = G.transpose() * (G * G.transpose()).inverse();  //不变的量
    Eigen::FullPivLU<Eigen::MatrixXd> lu(G);
    G_null_space = lu.kernel(); //不变量 G的一组基
    
}

Eigen::Matrix3d cable_load::cal_skew_matrix(Eigen::Vector3d x)
{
    Eigen::Matrix3d x_hat;
    x_hat << 0, -x(2), x(1),
    x(2), 0, -x(0),
    -x(1), x(0), 0; 
    return x_hat;
}