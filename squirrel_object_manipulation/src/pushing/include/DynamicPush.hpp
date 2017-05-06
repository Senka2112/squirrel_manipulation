#ifndef DYNAMICPUSH_H
#define DYNAMICPUSH_H

#include "PushPlanner.hpp"

#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>

class DynamicPush : public PushPlanner {

private:

    bool pub_values_;

    ros::Publisher velocity_pub_;
    ros::Publisher psi_push_pub_;
    ros::Publisher psi_rel_pub_;
    ros::Publisher compensate_pub_;
    ros::Publisher push_pub_;
    ros::Publisher rel_pub_;
    ros::Publisher sigma_alpha_pub_;
    ros::Publisher mi_alpha_pub_;

    control_toolbox::Pid pid_alpha_;
    geometry_msgs::Twist cmd;
    double p_alpha_, d_alpha_, i_alpha_, i_alpha_min_, i_alpha_max_;

    double filt_com;
    double aPOR, mi_alpha, sigma_alpha, aPORp;
    double gamma, gamma_old, mi_gamma, sigma_gamma;
    double beta, beta_old, mi_beta, sigma_beta;
    int count_dr, count_all;
    double psi_push_, psi_rel_;
    double expected_dir, executed_dir, result_dir;

    arma::mat data_cont_mat_;
    arma::vec alpha_vec, gamma_vec;


protected:
    void initChild();
    void updateChild();

public:

    DynamicPush();
    geometry_msgs::Twist getVelocities();
    void saveDataChild(string path);


};


#endif // DynamicPush_H
