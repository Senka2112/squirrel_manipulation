#include <ros/ros.h>
#include <iostream>
#include "squirrel_manipulation_msgs/GetObjectPositions.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_push_manipulation/conversion_utils.hpp>

#include <tf/transform_listener.h>

using namespace std;
std::string getexepath();

string global_frame_ ="map";

bool getPos(squirrel_manipulation_msgs::GetObjectPositions::Request  &req,
            squirrel_manipulation_msgs::GetObjectPositions::Response &res){


    ROS_INFO("Requested object positions");


    int obj_num = 3;
    //std::string objects[] = {"simple_blue_box","simple_red_box","orange_cylinder","blue_ball","green_cylinder","red_ball","simple_blue_box2","simple_yellow_box","simple_green_box","yellow_cylinder"};
    //double diameters[] = {0.40, 0.20, 0.340, 0.20, 0.20, 0.40, 0.40, 0.240, 0.20, 0.30};
    std::string objects[] = {"blue_ball","green_cylinder","simple_blue_box2"};
    double diameters[] = {0.30, 0.40, 0.45};


    res.diameters.resize(obj_num);
    res.objectids.resize(obj_num);
    res.objectposes.resize(obj_num);

    //tf::TransformListener tf_listener_;
    //for(int i = 0; i<obj_num; i++){
    //        res.diameters[i] = diameters[i];
    //        res.objectids[i] = objects[i];
    //        string object_id_ =  objects[i];
    //        tf::StampedTransform trans;
    geometry_msgs::PoseStamped pose_object_;

    string path = getexepath();
    //cout<<path<<endl;

    string p = "devel/squirrel_push_manipulation/lib/squirrel_push_manipulation/getobj";
    string base = path.substr(0, path.length() - p.length());
    string path_file = base+"src/squirrel_manipulation/data/object_locations.txt";
    //cout<<path_file<<endl;
    ifstream infile;
    infile.open(path_file.c_str());
    if (infile.is_open()){
        string line;
        std::string obj;
        double x, y, z;

        int line_count = 0;
        while (getline(infile,line)) {
            std::istringstream in(line);
            in >> obj >> x >> y >> z;
            pose_object_.pose.position.x = x;
            pose_object_.pose.position.y = y;
            pose_object_.pose.position.y = z;
            pose_object_.pose.orientation.w = 1.0;
            pose_object_.header.frame_id = "map";
            res.objectposes[line_count] = pose_object_.pose;
            res.diameters[line_count] = diameters[line_count];
            res.objectids[line_count] = objects[line_count];
            //cout<<pose_object_<<endl;
            line_count++;

        }
    }



    //start tracking
    //        squirrel_object_perception_msgs::StartObjectTracking srvStartTrack;
    //        srvStartTrack.request.object_id.data = object_id_;
    //        ROS_INFO("(Get poses) Waiting for the tracker of the %s to start \n", object_id_.c_str());

    //        if(!ros::service::call("/squirrel_start_object_tracking", srvStartTrack)){
    //            ROS_ERROR("(Get poses) Start tracking of the %s failed \n", object_id_.c_str());
    //        }

    //        try {
    //            tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(10.0));
    //            tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
    //            pose_object_ = tf_stamped2pose_stamped(trans);
    //        } catch (tf::TransformException& ex) {
    //            std::string ns = ros::this_node::getNamespace();
    //            std::string node_name = ros::this_node::getName();
    //            ROS_ERROR("(Get poses) %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
    //        }
    // res.objectposes[i] = pose_object_.pose;

    //stop tracking

    //        squirrel_object_perception_msgs::StopObjectTracking srvStopTrack;
    //        ros::service::call("/squirrel_stop_object_tracking", srvStopTrack);
    //        ROS_INFO("(Get poses) tracking of %s stopped \n", object_id_.c_str());

    //}
    ROS_INFO("Retrieving obejct locations finished");
    cout<<"result"<<endl<<res<<endl;

    return true;

}

std::string getexepath()
{
    char result[ PATH_MAX ];
    ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
    return std::string( result, (count > 0) ? count : 0 );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getObjectsPositions");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/getObjectsPositions", getPos);
    ROS_INFO("Ready to get object postion");
    ros::spin();
    return 0;
}
