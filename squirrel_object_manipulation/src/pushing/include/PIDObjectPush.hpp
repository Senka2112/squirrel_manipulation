#ifndef PIDOBJECTPUSH_H
#define PIDOBJECTPUSH_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>
#include <control_toolbox/pid.h>

class PIDObjectPush : public PushPlanner {

private:
    ros::NodeHandle nh;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    double p_x_, d_x_, i_x_, i_x_min_, i_x_max_;
    double p_y_, d_y_, i_y_, i_y_min_, i_y_max_;
    double p_theta_, d_theta_, i_theta_, i_theta_min_, i_theta_max_;

protected:
    void initChild();

public:

    PIDObjectPush();
    geometry_msgs::Twist getVelocities();


};


#endif // PIDOBJECTPUSH_H
