#ifndef DYNAMICPUSH_H
#define DYNAMICPUSH_H

#include "PushPlanner.hpp"


#include <geometry_msgs/Pose2D.h>
#include <control_toolbox/pid.h>

class DynamicPush : public PushPlanner {

private:
    ros::NodeHandle nh;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;
    control_toolbox::Pid pid_xd_;
    control_toolbox::Pid pid_yd_;
    int count;

    double aPOR;


    double p_x_, d_x_, i_x_, i_x_min_, i_x_max_;
    double p_y_, d_y_, i_y_, i_y_min_, i_y_max_;
    double p_theta_, d_theta_, i_theta_, i_theta_min_, i_theta_max_;

    arma::vec vx_p_, vy_p_, vdO2P_;
    arma::vec theta_vec, alpha_vec;
    double mi_dr, sigma_dr, dRlOTp, dO2Pp;
    double mi_theta, sigma_theta, aPORp;
    double mi_beta, sigma_beta, betap;
    int count_dr;
    double psi_push_, psi_rel_;

    double alpha, beta;


protected:
    void initChild();
    void updateChild();

public:

    DynamicPush();
    geometry_msgs::Twist getVelocities();


};


#endif // DynamicPush_H
