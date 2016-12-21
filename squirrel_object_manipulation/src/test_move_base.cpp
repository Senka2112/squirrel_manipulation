#include <ros/ros.h>
#include <iostream>

#include <squirrel_object_manipulation/RobotinoBaseControl.hpp>


using namespace std;

int main(int argc, char** args) {

    ros::init(argc, args, "base_move_test");
    ros::NodeHandle node;
    usleep(1e6);

    RobotinoBaseControl robotino(node, 20.0, 0.6, 0.3, 0.3);
    ros::Rate lRate(20.0);

    double target = robotino.getCurrentState();
    vector<double> current_pose = robotino.getCurrentPose();
    double cx = current_pose.at(1);
    double cy = current_pose.at(2);

    cout <<"start pose "<<current_pose.at(0)<<endl<<current_pose.at(1)<<endl<<current_pose.at(2)<<endl;

    while (ros::ok){

        //vector<double> current_pose = robotino.getCurrentPose();
        // cout <<"current pose "<<current_pose.at(0)<<endl<<current_pose.at(1)<<endl<<current_pose.at(2)<<endl;

       // cout<<"current state "<<robotino.getCurrentPose()<<endl;
       // target = target - 0.05;
       // cx = cx - 0.01;
       // cy = cy - 0.01;
       robotino.move(target, cx + 0.2, cy + 0.2);
       // robotino.move(current_pose.at(2) + 0.2);
        lRate.sleep();


    }


}
