#include <squirrel_object_manipulation/pushing.hpp>

#include <nav_msgs/Odometry.h>
#include <robotino_msgs/ResetOdometry.h>


using namespace std;

bool nav = false;


bool tag = true ;
bool firstSet = false;
void arCallback(tf::tfMessage msg);
geometry_msgs::TransformStamped t, tm1,tm2;
int count_tr = 0;
double dro = 0.21 + 0.1;
double offsetX, offsetY; // ar tags calibration. inital offset



PushAction::PushAction(const std::string std_PushServerActionName) :
    pushServer(nh, std_PushServerActionName , boost::bind(&PushAction::executePush, this, _1), false),
    private_nh("~"),
    runPushPlan_(false),
    trackingStart_(false),
    objectLost_(false),
    first_pose_(false)
{
    node_name_ = ros::this_node::getName();

    private_nh.param("pose_topic", pose_topic_,std::string("/squirrel_localizer_pose"));
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_frame", global_frame_, std::string("/map"));
    private_nh.param("controller_frequency", controller_frequency_, 20.00);
    private_nh.param("tilt_nav", tilt_nav_, 0.60);
    private_nh.param("tilt_perception", tilt_perception_, 1.1);
    private_nh.param("lookahead", lookahead_, 0.30);
    private_nh.param("goal_tolerance", goal_toll_, 0.20);
    private_nh.param("state_machine", state_machine_, false);
    private_nh.param("object_diameter", object_diameter_, 0.2);
    private_nh.param("robot_diameter", robot_diameter_, 0.42);
    private_nh.param("corridor_width", corridor_width_ , 1.2);
    //private_nh.param("push_planner", push_planner_, new PushPlanner());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new SimplePathFollowing());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new SimplePush());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new BangBangPush());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new PIDPush());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new PIDSimplePush());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new PIDObjectPush());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new DipoleField());
    //push_planner_ = boost::shared_ptr<PushPlanner>(new CentroidAlignment());
    push_planner_ = boost::shared_ptr<PushPlanner>(new DynamicPush());

    //set callback for cancel request
    pushServer.registerPreemptCallback(boost::bind(&PushAction::preemptCB, this));

    pose_sub_ = nh.subscribe(pose_topic_, 2, &PushAction::updatePose, this);
    robotino = boost::shared_ptr<RobotinoControl>(new RobotinoControl(nh));

    object_tracking_thread_ = new boost::thread(boost::bind(&PushAction::objectTrackingThread, this));

    pushServer.start();
    ROS_INFO("(Push) Ready to push objects");
    cout << endl;
}

PushAction::~PushAction() {

    object_tracking_thread_->interrupt();
    object_tracking_thread_->join();
    robotino->stopRobot();

    delete object_tracking_thread_;
}

void PushAction::executePush(const squirrel_manipulation_msgs::PushGoalConstPtr &goal) {

    ros::Subscriber  markerSub = nh.subscribe("/tf2", 10, arCallback);

    if(!nav){
        ros::ServiceClient rO=nh.serviceClient<robotino_msgs::ResetOdometry>("/reset_odometry");
        robotino_msgs::ResetOdometry R;
        R.request.x=0;
        R.request.y=0;
        R.request.phi=0;

        if( rO.call(R))
        {cout<<"call success"<<endl;}
        else{ cout<<"call not successful"<<endl; }
    }


    ROS_INFO("(Push) Started pushing of %s  \n",  goal->object_id.c_str());
    cout << endl;

    // initilize push
    runPushPlan_ = true;
    trackingStart_ = false;
    objectLost_ = false;

    //set controller rate
    ros::Rate lRate(controller_frequency_);

    //get goal
    push_goal_.pose = goal->pose;
    object_id_ = goal->object_id;


    //get the object diameter

    //    mongodb_store::MessageStoreProxy message_store(nh);

    //    //get the object diameter

    //    // fetch position of object from message store
    //    std::vector< boost::shared_ptr<std_msgs::Float64> > results;

    //    if(message_store.queryNamed<std_msgs::Float64>(object_id_, results)) {
    //        cout << "matching objects to '" << object_id_ << "'\n";
    //        for(size_t i = 0; i < results.size(); i++)
    //            cout << "  " << *results[i] << "\n";
    //        if(results.size()<1) {
    //            ROS_ERROR(" no matching obID %s", object_id_.c_str());
    //            return;
    //        }
    //        if(results.size()>1)
    //            ROS_ERROR("(Push)  multiple objects share the same wpID");
    //    } else {
    //        ROS_ERROR("(Push) could not query message store to fetch object size");
    //        return;
    //    }
    //    std_msgs::Float64 val = *results[0];

    //    sleep (5.0);
    //    cout << "out of datab"<<endl;
    //    cout << "val "<<val<<endl;



    //object_diameter_ = *results[0];



    if(!isQuaternionValid(goal->pose.orientation)){
        ROS_INFO("(Push): Invalid target orientation \n");
        cout << endl;
        abortPush();
        return;
    }

    ros::spinOnce();


    // start object tracking
    // move camera for vision
    //robotino->moveTilt(tilt_perception_);
    sleep (5.0);
    if(startTracking()){
        ROS_INFO("(Push) Waiting for the tracker of the %s to start \n",  goal->object_id.c_str());
        trackingStart_ = true;
    }
    else{
        ROS_ERROR("(Push) Start tracking of the %s failed \n",  goal->object_id.c_str());
        abortPush();
        return;
    }
    cout << endl;

    if(getFirstObjectPose()){
        ROS_INFO("(Push) Tracking started. \n",  goal->object_id.c_str());
    }
    else{
        ROS_ERROR("(Push) Getting first pose failed \n" ,  goal->object_id.c_str());
        abortPush();
        return;
    }
    cout << endl;

    //getting path from navigation
    if(!getPushPath()){
        ROS_ERROR("(Push) Getting a path from navigation failed \n");
        abortPush();
        return;
    }
    cout << endl;

    //initialize push planner
    if (runPushPlan_){
        push_planner_->initialize(robot_base_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_, goal_toll_, state_machine_, controller_frequency_, object_diameter_, robot_diameter_, corridor_width_);
        push_planner_->visualisationOn();
        push_planner_->startPush();
    }

    double secs =ros::Time::now().toSec();

    try{
        //main push loop
        while (nh.ok() &&  push_planner_->push_active_  && runPushPlan_ ){

            push_planner_->updatePushPlanner(pose_robot_, pose_object_);
            geometry_msgs::Twist cmd = push_planner_->getControlCommand();
            // cout<<cmd<<endl;
            robotino->singleMove(cmd.linear.x, cmd.linear.y,0.0,0.0,0.0,cmd.angular.z);

            lRate.sleep();

        }
    }
    catch (...){
    }
    push_planner_->setExperimentName("straightD120real");
    push_planner_->saveData("/home/c7031098/squirrel_ws_new/data/");

    if (push_planner_->goal_reached_){
        ROS_INFO("Goal reached sucessfully \n");
        finishSuccess();
        return;
    }else{
        //if pushing did not result with success for any reason
        abortPush();
    }
    double secs2 =ros::Time::now().toSec();
    cout<<"push duration "<<secs2 - secs;

    cout << endl;

    //end of action instance


    return;


}

bool PushAction::getFirstObjectPose(){

    first_pose_ = false;
    tf::StampedTransform trans;

    if(!tag){

        if(trackingStart_&&(!first_pose_)){
            try {
                tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(10.0));
                tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
                pose_object_ = tf_stamped2pose_stamped(trans);
                first_pose_ = true;
            } catch (tf::TransformException& ex) {
                std::string ns = ros::this_node::getNamespace();
                std::string node_name = ros::this_node::getName();
                ROS_ERROR("(Push) %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
            }

        }

        return first_pose_;
    }
    else return true;

}

void PushAction::objectTrackingThread(){

    ros::Rate lRate(controller_frequency_);
    ros::NodeHandle n;

    tf::StampedTransform trans;
    first_pose_ = false;

    while (n.ok()){


        if(!nav){

            robot_pose_mutex_.lock();
            nav_msgs::Odometry Odometry = robotino->getOdom();;
            //robotino position in /odom world
            pose_robot_.x = Odometry.pose.pose.position.x;
            pose_robot_.y = Odometry.pose.pose.position.y;
            pose_robot_.theta = tf::getYaw(Odometry.pose.pose.orientation);
            robot_pose_mutex_.unlock();
        }

        object_pose_mutex_.lock();
        lRate.sleep();

        if(!tag){

            if(trackingStart_&&first_pose_){
                try {
                    tf_listener_.waitForTransform(global_frame_, object_id_, ros::Time::now(), ros::Duration(0.2));
                    tf_listener_.lookupTransform(global_frame_, object_id_, ros::Time(0), trans);
                    pose_object_ = tf_stamped2pose_stamped(trans);
                } catch (tf::TransformException& ex) {
                    std::string ns = ros::this_node::getNamespace();
                    std::string node_name = ros::this_node::getName();
                    ROS_ERROR("(Push) %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());
                    abortPush();
                }
            }
        }

        if(tag&firstSet){

            pose_object_.header.frame_id =  global_frame_;

            if (count_tr < 2) {
                tm2 = t;
                tm1 = t;}

            double Olx=(t.transform.translation.x + tm1.transform.translation.x+ tm2.transform.translation.x) / 3 -offsetX;
            double Oly= (t.transform.translation.y + tm1.transform.translation.y+ tm2.transform.translation.y) / 3 -offsetY;
            double th = pose_robot_.theta;

            pose_object_.pose.position.x = pose_robot_.x - Oly*cos(th)+Olx*sin(th);;
            pose_object_.pose.position.y = pose_robot_.y - Oly*sin(th)-Olx*cos(th);
            pose_object_.pose.position.z = 0;
            pose_object_.pose.orientation = t.transform.rotation;

            if (count_tr > 2) {
                tm2 = tm1;
                tm1 = t;}
            count_tr ++;

            cout<<" Olx "<<Olx<<" Oly "<< Oly<<" off x "<<offsetX<<" off Y "<<offsetY<<endl;

            cout<<pose_object_<<endl;
        }
        object_pose_mutex_.unlock();
    }
}


void PushAction::updatePose( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg ){

    if(nav){

        robot_pose_mutex_.lock();
        pose_robot_.x = pose_msg->pose.pose.position.x;
        pose_robot_.y = pose_msg->pose.pose.position.y;
        pose_robot_.theta = tf::getYaw(pose_msg->pose.pose.orientation);
        robot_pose_mutex_.unlock();
    }
}


bool PushAction::getPushPath(){

    if (nav){

        squirrel_rgbd_mapping_msgs::GetPushingPlan srvPlan;

        // part which has to be revised
        geometry_msgs::PoseStamped start_m;
        try {
            tfl_.waitForTransform(global_frame_, "/map", ros::Time::now(), ros::Duration(1.0));
            tfl_.transformPose("/map", pose_object_, start_m);
            start_m.header.frame_id = "/map";
        } catch ( tf::TransformException& ex ) {
            ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
            return true;
        }

        // Getting pushing plan
        srvPlan.request.start.x =  start_m.pose.position.x;
        srvPlan.request.start.y =  start_m.pose.position.y;
        srvPlan.request.start.theta =  pose_robot_.theta;

        srvPlan.request.goal.x = push_goal_.pose.position.x;
        srvPlan.request.goal.y = push_goal_.pose.position.y;
        srvPlan.request.goal.theta = tf::getYaw(push_goal_.pose.orientation);

        // Set object polygon
        geometry_msgs::Point32 p1, p2, p3, p4;

        p1.x = start_m.pose.position.x + object_diameter_; p1.y = start_m.pose.position.y - object_diameter_;
        p2.x = start_m.pose.position.x + object_diameter_; p2.y = start_m.pose.position.y + object_diameter_;
        p3.x = start_m.pose.position.x - object_diameter_; p3.y = start_m.pose.position.y - object_diameter_;
        p4.x = start_m.pose.position.x - object_diameter_; p4.y = start_m.pose.position.y + object_diameter_;


        // Make request

        srvPlan.request.object.points.push_back(p1);
        srvPlan.request.object.points.push_back(p2);
        srvPlan.request.object.points.push_back(p3);
        srvPlan.request.object.points.push_back(p4);

        if ( ros::service::call("/getPushingPlan", srvPlan) ) {
            if ( srvPlan.response.plan.poses.empty() ) {
                ROS_WARN("(Push) Got an empty plan");

            } else {
                BOOST_ASSERT_MSG( srvPlan.response.plan.header.frame_id == "/map" ||
                                  srvPlan.response.plan.header.frame_id == "map" ,
                                  "returned path is not in requested frame");
            }
        } else {
            ROS_ERROR("(Push) unable to communicate with /getPushingPlan");
            return false;
        }

        // Convert plan to global_frame_
        pushing_path_.header.frame_id = global_frame_;
        for (unsigned int i=0; i<srvPlan.response.plan.poses.size(); ++i) {
            try {
                geometry_msgs::PoseStamped p;
                tfl_.waitForTransform("/map", global_frame_, ros::Time::now(), ros::Duration(0.5));
                tfl_.transformPose(global_frame_, srvPlan.response.plan.poses[i], p);
                p.header.frame_id = global_frame_;
                if (pushing_path_.poses.size() > 1){
                    geometry_msgs::PoseStamped pom = pushing_path_.poses[pushing_path_.poses.size() - 1];
                    pom.pose.position.x = (pom.pose.position.x + p.pose.position.x) / 2;
                    pom.pose.position.y = (pom.pose.position.y + p.pose.position.y) / 2;
                    pushing_path_.poses.push_back(pom);
                }
                pushing_path_.poses.push_back(p);
            } catch (tf::TransformException& ex) {
                ROS_ERROR("%s: %s", node_name_.c_str(), ex.what());
                return false;
            }
        }
    }
    else{
        pushing_path_.header.frame_id = global_frame_;
        int size = 100;
        double x_max = 1.5;
        for (unsigned int i=0; i<size; ++i) {

            geometry_msgs::PoseStamped p;

            p.header.frame_id = global_frame_;
            p.pose.position.x = x_max/size * i;
            p.pose.position.y = 0.0;

            pushing_path_.poses.push_back(p);

        }
    }

    ROS_INFO("(Push) Path ready for pushing \n");


    return true;

}

void PushAction::abortPush(){

    ROS_INFO("(Push) Aborting push action \n");

    pushResult.result_status = "failure";
    pushServer.setAborted(pushResult);
    finishPush();

}

void PushAction::finishPush(){

    if(trackingStart_){
        if (stopTracking()){
            ROS_INFO("(Push) Object tracking stopped \n");
        }
        else{
            ROS_ERROR("(Push) Object tracking did not finish properly \n");
        }
    }

    trackingStart_ = false;
    runPushPlan_ = false;
    pushing_path_.poses.clear();


    //moving tilt for navigation configuration
    //robotino->moveTilt(tilt_nav_);
    ros::spinOnce();

}
void PushAction::finishSuccess(){

    ROS_INFO("(Push) Push action executed sucessfully \n");

    pushResult.result_status = "success";
    pushServer.setSucceeded(pushResult);
    finishPush();

}

bool PushAction::startTracking() {
    if(!tag){

        squirrel_object_perception_msgs::StartObjectTracking srvStartTrack;
        srvStartTrack.request.object_id.data = object_id_;
        return(ros::service::call("/squirrel_start_object_tracking", srvStartTrack));
    }
    else{
        while(!firstSet){
            cout<<"wait for first set"<<endl;
        }
        return firstSet;
    }
}

bool PushAction::stopTracking() {
    bool track = true;
    if(!tag){
        squirrel_object_perception_msgs::StopObjectTracking srvStopTrack;
        track = ros::service::call("/squirrel_stop_object_tracking", srvStopTrack);
        trackingStart_ = false;
        first_pose_ = false;
        objectLost_ = false;
    }
    else firstSet = false;

    return track;
}


void PushAction::preemptCB(){

    ROS_INFO("(Push) Canceled push action by the high-level planner");
    cout<<endl;
    runPushPlan_ = false;
    //this->finishPush();
    // pushServer.setPreempted();


}

int main(int argc, char** argv) {


    ros::init(argc, argv, "base_pushing");

    PushAction push(PUSH_NAME);
    ros::spin();

    return 0;

}

void arCallback(tf::tfMessage msg) {
    t = msg.transforms.at(0);

    if (!firstSet){
        firstSet = true;
        offsetX = t.transform.translation.x;
        offsetY = t.transform.translation.y + dro;

    }

}


