#include<iostream>
using namespace std;
#include <vector>
#include <Eigen/Eigen> 

Eigen::Matrix3d cal_skew_matrix(Eigen::Vector3d x)
{
    Eigen::Matrix3d x_hat;
    x_hat << 0, -x(2), x(1),
    x(2), 0, -x(0),
    -x(1), x(0), 0; 
    return x_hat;
}

int main()
{
    // Eigen::Vector3d acc={3,2,1};
    // Eigen::Matrix3d x_hat;
    // cal_skew_matrix(acc, x_hat);
    // cout<<x_hat;

    Eigen::Vector3d acc={3,2,1};
    double m_load = 2.0;
    Eigen::Matrix<double, 6, 1> FM;
    FM <<  m_load*acc, 0.0, 0.0, 0.0;
    // cout<<FM;

    //test
    std::vector<Eigen::Vector3d> cable_points;
    double length = 1.0, width = 0.5, height = 0.5;
    double half_length = length/2.0, half_width = width/2.0, half_height = height/2.0;
    cable_points.push_back({-half_length, -half_width, -half_height});
    cable_points.push_back({-half_length, half_width, -half_height});
    cable_points.push_back({half_length, half_width, -half_height});
    cable_points.push_back({half_length, -half_width, -half_height});

    Eigen::MatrixXd G(6,12);
    for (size_t i = 0; i < cable_points.size(); i++)
    {
        G.block<3,3>(0,3*i) << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

        G.block<3,3>(3,3*i) = cal_skew_matrix(cable_points[i]);
    }

    Eigen::Matrix<double, 12, 6> G_inv = G.transpose() * (G * G.transpose()).inverse();
    
    Eigen::FullPivLU<Eigen::MatrixXd> lu(G);
    Eigen::MatrixXd G_null_space = lu.kernel();
    Eigen::MatrixXd G_random(6,1);
    G_random << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::MatrixXd FM_each = G_null_space*G_random + G_inv*FM;
    
    std::cout << G*FM_each<<endl;

    


    return 0;
}