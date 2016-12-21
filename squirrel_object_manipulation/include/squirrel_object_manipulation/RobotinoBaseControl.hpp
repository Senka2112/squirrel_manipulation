#ifndef ROBOTINOBASECONTROL
#define ROBOTINOBASECONTROL

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <control_toolbox/pid.h>
#include <tf/tf.h>
#include <vector>

#define ROBOTINO_MOVE_TOPIC "/cmd_vel"
#define ROBOTINO_ODOM_TOPIC "/odom"

class RobotinoBaseControl {

private:

 //   static auto constexpr ROBOTINO_MOVE_TOPIC = "/cmd_vel";
 //   static auto constexpr ROBOTINO_ODOM_TOPIC = "/odom";

    ros::NodeHandle  private_nh;

    ros::Subscriber subOdometry;
    ros::Publisher pubMove;

    nav_msgs::Odometry odometry;

    boost::thread* move_base_thread_;
    bool start_move_base_;
    bool move_base_3dof_;

    double controller_frequency_, time_step_;
    double vel_ang_max_, vel_x_max_, vel_y_max_;
    double p_theta_, d_theta_, i_theta_, i_theta_min_, i_theta_max_;
    double p_x_, d_x_, i_x_, i_x_min_, i_x_max_;
    double p_y_, d_y_, i_y_, i_y_min_, i_y_max_;

    double desired_theta_, desired_x_, desired_y_;

    control_toolbox::Pid pid_theta_;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    geometry_msgs::Twist current_base_vel_;

    boost::mutex robot_pose_mutex_;
    boost::mutex move_pose_mutex_;

    void callbackOdometry(nav_msgs::Odometry msg);
    void moveBaseThread();

    geometry_msgs::Twist getNullTwist();
    double rotationDifference(double angle, double theta_robot);

    void initialize(ros::NodeHandle& node, double controller_frequency_);


public:

    RobotinoBaseControl(ros::NodeHandle& node, double  controller_frequency_, double max_ang_vel_);
    RobotinoBaseControl(ros::NodeHandle& node, double  controller_frequency_, double max_ang_vel_, double vel_x_max_, double vel_y_max_);
    ~RobotinoBaseControl();

    void move(double desired_theta);
    void move(double desired_theta, double desired_x, double desired_y);

    double getCurrentState();
    std::vector<double> getCurrentPose();

};

#endif
