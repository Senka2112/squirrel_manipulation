#include "squirrel_object_manipulation/RobotinoBaseControl.hpp"

//#include <chrono>
//#include <thread>

using namespace ros;
using namespace std;

void RobotinoBaseControl::initialize(ros::NodeHandle& node, double controller_frequency_){
    
    private_nh.param("baseControl/proportional_theta", p_theta_, 0.6);
    private_nh.param("baseControl/derivative_theta", d_theta_, 0.4);
    private_nh.param("baseControl/integral_theta", i_theta_, 0.0);
    private_nh.param("baseControl/integral_theta_max", i_theta_max_, 0.8);
    private_nh.param("baseControl/integral_theta_min", i_theta_min_, -0.8);
    
    private_nh.param("baseControl/proportional_x", p_x_, 0.6);
    private_nh.param("baseControl/derivative_x", d_x_, 0.4);
    private_nh.param("baseControl/integral_x", i_x_, 0.0);
    private_nh.param("baseControl/integral_x_max", i_x_max_, 0.8);
    private_nh.param("baseControl/integral_x_min", i_x_min_, -0.8);
    
    private_nh.param("baseControl/proportional_y", p_y_, 0.6);
    private_nh.param("baseControl/derivative_y", d_y_, 0.4);
    private_nh.param("baseControl/integral_y", i_y_, 0.0);
    private_nh.param("baseControl/integral_y_max", i_y_max_, 0.8);
    private_nh.param("baseControl/integral_y_min", i_y_min_, -0.8);
    
    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
    pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_y_.initPid(p_y_, i_y_, d_y_, i_y_max_, i_y_min_);
    
    this->time_step_ = 1 / controller_frequency_;
    
    subOdometry = node.subscribe(ROBOTINO_ODOM_TOPIC, 1, &RobotinoBaseControl::callbackOdometry, this);
    pubMove = node.advertise<geometry_msgs::Twist>(ROBOTINO_MOVE_TOPIC, 1);
    
    sleep(1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    move_base_thread_ = new boost::thread(boost::bind(&RobotinoBaseControl::moveBaseThread, this));

}

RobotinoBaseControl::RobotinoBaseControl(ros::NodeHandle& node, double controller_frequency_, double vel_ang_max_): controller_frequency_(controller_frequency_), vel_ang_max_(vel_ang_max_), start_move_base_(false), move_base_3dof_(false), private_nh("~") {
    
    this->initialize(node, controller_frequency_);
}

RobotinoBaseControl::RobotinoBaseControl(ros::NodeHandle& node, double controller_frequency_, double vel_ang_max_, double vel_x_max_, double vel_y_max_): controller_frequency_(controller_frequency_), vel_ang_max_(vel_ang_max_), vel_x_max_(vel_x_max_), vel_y_max_(vel_y_max_), start_move_base_(false), move_base_3dof_(false), private_nh("~") {

    this->initialize(node, controller_frequency_);
}

RobotinoBaseControl::~RobotinoBaseControl() {
    
    move_base_thread_->interrupt();
    move_base_thread_->join();
    
    delete move_base_thread_;
}


void RobotinoBaseControl::callbackOdometry(nav_msgs::Odometry msg) {
    robot_pose_mutex_.lock();
    odometry = msg;
    robot_pose_mutex_.unlock();
}

void RobotinoBaseControl::move(double desired_theta) {
    
    move_pose_mutex_.lock();
    desired_theta_ = desired_theta;
    start_move_base_ = true;
    move_base_3dof_ = false;
    move_pose_mutex_.unlock();
    
}

void RobotinoBaseControl::move(double desired_theta, double desired_x, double desired_y) {
    
    move_pose_mutex_.lock();
    desired_theta_ = desired_theta;
    desired_x_ = desired_x;
    desired_y_ = desired_y;
    start_move_base_ = true;
    move_base_3dof_ = true;
    move_pose_mutex_.unlock();
    
}


void RobotinoBaseControl::moveBaseThread(){
    
    ros::Rate moveBaseRate(controller_frequency_);
    ros::spinOnce();
    
    while (ros::ok){
        
        if(start_move_base_) {
            
            current_base_vel_ = getNullTwist();
            double current_theta = tf::getYaw(odometry.pose.pose.orientation);
            double orient_error = rotationDifference(desired_theta_, current_theta);
            current_base_vel_.angular.z = pid_theta_.computeCommand(orient_error, ros::Duration(time_step_));
            if(fabs(current_base_vel_.angular.z) > vel_ang_max_) current_base_vel_.angular.z = (current_base_vel_.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);
            
            if(move_base_3dof_){
                cout<<"desired x "<<desired_x_<<endl;
                cout<<" current x "<<odometry.pose.pose.position.x<<endl;
                cout<<" error x"<<desired_x_ - odometry.pose.pose.position.x<<endl;
                double err_x_odom = desired_x_ - odometry.pose.pose.position.x;
                double err_y_odom = desired_y_ - odometry.pose.pose.position.y;

                double err_x_r = cos(current_theta) * err_x_odom - sin(current_theta) * err_y_odom;
                double err_y_r = sin(current_theta) * err_x_odom + cos(current_theta) * err_y_odom;

                current_base_vel_.linear.x = pid_x_.computeCommand(err_x_r, ros::Duration(time_step_));
                if(fabs(current_base_vel_.linear.x) > vel_x_max_) current_base_vel_.linear.x= (current_base_vel_.linear.x > 0 ? vel_x_max_ : - vel_x_max_);

                cout<<" error y"<<desired_y_ - odometry.pose.pose.position.y<<endl;
                current_base_vel_.linear.y = pid_y_.computeCommand(err_y_r, ros::Duration(time_step_));
                if(fabs(current_base_vel_.linear.y) > vel_y_max_) current_base_vel_.linear.y = (current_base_vel_.linear.y > 0 ? vel_y_max_ : - vel_y_max_);

            }
            
            pubMove.publish(current_base_vel_);
            start_move_base_ = false;
        }
        
        moveBaseRate.sleep();
        ros::spinOnce();
    }
    
}

geometry_msgs::Twist RobotinoBaseControl::getNullTwist() {
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    
    return cmd;
}

double RobotinoBaseControl::rotationDifference(double angle, double theta_robot) {
    
    double err_th = angle - theta_robot;
    
    if(err_th > M_PI) err_th = - (2 * M_PI - err_th);
    if(err_th < -M_PI) err_th = 2 * M_PI + err_th;
    
    return err_th;
}

vector<double>  RobotinoBaseControl::getCurrentPose() {
    
    robot_pose_mutex_.lock();
    geometry_msgs::Pose odomPose = odometry.pose.pose;
    robot_pose_mutex_.unlock();
    
    vector<double> current_pose;
    current_pose.push_back(odomPose.position.x);
    current_pose.push_back(odomPose.position.y);
    current_pose.push_back(tf::getYaw(odomPose.orientation));

    return current_pose;
    
}


double RobotinoBaseControl::getCurrentState() {

    robot_pose_mutex_.lock();
    geometry_msgs::Quaternion odomBkp = odometry.pose.pose.orientation;
    robot_pose_mutex_.unlock();

    return tf::getYaw(odomBkp);

}
