#include "../include/tf_nav.h"

void TF_NAV::arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    tf::Vector3 ArucoPosition(
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
    tf::Quaternion ArucoOrientation(
        msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
    _tfAruco = tf::Transform(ArucoOrientation,ArucoPosition);
}

TF_NAV::TF_NAV(bool allowExploration, const int totalNumberOfGoals)
        : _totalNumberOfGoals(totalNumberOfGoals) {

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    // subscribe to the aruco single pose topic
    _aruco_pose_sub = _nh.subscribe("/aruco_single/pose", 1, &TF_NAV::arucoPoseCallback, this);

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _goal_pos.resize(_totalNumberOfGoals);
    _goal_or.resize(_totalNumberOfGoals);
    for (int goal_number = 0; goal_number < _totalNumberOfGoals; ++goal_number) {
        _goal_pos.at(goal_number) << 0.0, 0.0, 0.0;
        _goal_or.at(goal_number) << 0.0, 0.0, 0.0, 1.0;
    }
    // updated home position according to the homework
    _home_pos << -3.0, 5.0, 0.0;
    _home_rot << 0, 0, -0.7068252, 0.7073883;   // yaw = -90°
    _allowExploration = allowExploration;

    // acquires the transformation matrix between camera and base_footprint
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform tfBaseCamera;

    try {
        listener.waitForTransform( "base_footprint", "camera_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform( "base_footprint", "camera_link", ros::Time(0), tfBaseCamera );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR("%s", ex.what());
        r.sleep();
        return;
    }

    _tfBaseCamera = tfBaseCamera;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );

        } catch ( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }

        _tfBase = transform;    // store the tf matrix of base footprint into object data
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }

}

void TF_NAV::position_pub() {

    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];

    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];

    _position_pub.publish(pose);
}

void TF_NAV::goal_listener() {
    ros::Rate r( 1 );
    //std::vector<tf::TransformListener*> listener;
    tf::TransformListener listener;
    std::vector<tf::StampedTransform*> transform;
    tf::StampedTransform tfGoalAruco;
    //listener.resize(_totalNumberOfGoals);
    transform.resize(_totalNumberOfGoals);
    for (int i = 0; i < _totalNumberOfGoals; ++i) {
        //listener[i] = new tf::TransformListener();
        transform[i] = new tf::StampedTransform();
    }

    static std::vector<bool> hasLogged;
    hasLogged.resize(_totalNumberOfGoals);
    for (int i = 0; i < _totalNumberOfGoals; ++i) {
        hasLogged[i] = false;
    }

    while ( ros::ok() ) {
        // for each of the goals, a listener waits for the tf publisher. Then the transform stores the position and orientation.
        // if it fails, the catch block skips to the next iteration.
        for (int goal_number = 0; goal_number < _totalNumberOfGoals; ++goal_number) {
            try {
                listener.waitForTransform( "map", "goal_frame_" + std::to_string(goal_number + 1), ros::Time( 0 ), ros::Duration( 10.0 ) );
                listener.lookupTransform("map", "goal_frame_" + std::to_string(goal_number + 1), ros::Time( 0 ),
                                                       *(transform[goal_number]));
                if (!hasLogged[goal_number]) {
                    ROS_INFO("transform[%d]: pos (%f, %f, %f), rot (%f, %f, %f, %f)\n", goal_number,
                             transform[goal_number]->getOrigin().x(),
                             transform[goal_number]->getOrigin().y(),
                             transform[goal_number]->getOrigin().z(),
                             transform[goal_number]->getRotation().x(),
                             transform[goal_number]->getRotation().y(),
                             transform[goal_number]->getRotation().z(),
                             transform[goal_number]->getRotation().w());
                    hasLogged[goal_number] = true;
                }

                _goal_pos.at(goal_number) << transform[goal_number]->getOrigin().x(), transform[goal_number]->getOrigin().y(), transform[goal_number]->getOrigin().z();
                _goal_or.at(goal_number) << transform[goal_number]->getRotation().w(),  transform[goal_number]->getRotation().x(), transform[goal_number]->getRotation().y(), transform[goal_number]->getRotation().z();

            } catch( tf::TransformException &ex ) {
                ROS_ERROR("goal_number = %d: %s", goal_number, ex.what());
                r.sleep();
                continue;
            }
        }

        try {
            listener.waitForTransform( "map", "goal_marker_frame", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform("map", "goal_marker_frame", ros::Time( 0 ),
                                     tfGoalAruco);

            _aruco_goal_pos << tfGoalAruco.getOrigin().x(), tfGoalAruco.getOrigin().y(), tfGoalAruco.getOrigin().z();
            _aruco_goal_or << tfGoalAruco.getRotation().w(),  tfGoalAruco.getRotation().x(), tfGoalAruco.getRotation().y(), tfGoalAruco.getRotation().z();

        } catch( tf::TransformException &ex ) {
            ROS_ERROR("goal_aruco %s", ex.what());
            r.sleep();
            continue;
        }

        r.sleep();
    }
}

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() )
    {
        std::cout<<"\nInsert 1 to send goal from TF ";
        if (_allowExploration) {
            std::cout << "for exploration";
        }
        std::cout<<"\nInsert 2 to send home position goal ";
        std::cout<<"\nInsert 3 to send aruco marker goal "<<std::endl;
        std::cout<<"Insert your choice"<<std::endl;
        std::cin>>cmd;

        if ( cmd == 1) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }

            for (int goal_index = 0; goal_index < _totalNumberOfGoals; ++goal_index) {
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();

                if (!_allowExploration) {
                    // if not in exploration mode, the order of the goals is the one specified in class header
                    goal.target_pose.pose.position.x = _goal_pos.at(goalOrder[goal_index] - 1)[0];
                    goal.target_pose.pose.position.y = _goal_pos.at(goalOrder[goal_index] - 1)[1];
                    goal.target_pose.pose.position.z = _goal_pos.at(goalOrder[goal_index] - 1)[2];

                    goal.target_pose.pose.orientation.w = _goal_or.at(goalOrder[goal_index] - 1)[0];
                    goal.target_pose.pose.orientation.x = _goal_or.at(goalOrder[goal_index] - 1)[1];
                    goal.target_pose.pose.orientation.y = _goal_or.at(goalOrder[goal_index] - 1)[2];
                    goal.target_pose.pose.orientation.z = _goal_or.at(goalOrder[goal_index] - 1)[3];

                    ROS_INFO("Sending goal %d", goalOrder[goal_index]);
                } else {
                    // if in exploration mode, the order is 1, ..., _totalNumberOfGoals
                    goal.target_pose.pose.position.x = _goal_pos.at(goal_index)[0];
                    goal.target_pose.pose.position.y = _goal_pos.at(goal_index)[1];
                    goal.target_pose.pose.position.z = _goal_pos.at(goal_index)[2];

                    goal.target_pose.pose.orientation.w = _goal_or.at(goal_index)[0];
                    goal.target_pose.pose.orientation.x = _goal_or.at(goal_index)[1];
                    goal.target_pose.pose.orientation.y = _goal_or.at(goal_index)[2];
                    goal.target_pose.pose.orientation.z = _goal_or.at(goal_index)[3];

                    ROS_INFO("Sending goal %d", goal_index);
                }
                ac.sendGoal(goal);

                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    // conditional operator to print the correct message
                    ROS_INFO("The mobile robot arrived in the TF goal number %d", (!_allowExploration) ? goalOrder[goal_index] : goal_index);
                } else {
                    ROS_INFO("The base failed to move for some reason");
                    // skip to the next iteration (skip to the next goal)
                    continue;
                }
            }
        } else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = _home_rot[3];
            goal.target_pose.pose.orientation.x = _home_rot[0];
            goal.target_pose.pose.orientation.y = _home_rot[1];
            goal.target_pose.pose.orientation.z = _home_rot[2];

            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move for some reason");
        } else if(cmd == 3) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            // First, send the robot near the Aruco marker, so it can look at the marker
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = _aruco_goal_pos[0];
            goal.target_pose.pose.position.y = _aruco_goal_pos[1];
            goal.target_pose.pose.position.z = _aruco_goal_pos[2];
            goal.target_pose.pose.orientation.w = _aruco_goal_or[0];
            goal.target_pose.pose.orientation.x = _aruco_goal_or[1];
            goal.target_pose.pose.orientation.y = _aruco_goal_or[2];
            goal.target_pose.pose.orientation.z = _aruco_goal_or[3];



            ROS_INFO("Sending Aruco as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived in the Aruco position");
            else{
                ROS_INFO("The base failed to move for some reason");
                return;
            }
            // At this point, the robot the Aruco pose is available and the robot has to move 1 meter in front of it
            tf::Matrix3x3 rotOffset(0,0,-1,-1,0,0,0,1,0);
            tf::Vector3 posOffset(0,0,1);
            tf::Transform tfOffset(rotOffset,posOffset);

            tf::Transform tfDesiredPose = _tfBase*_tfBaseCamera*_tfAruco*tfOffset;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            // goal.target_pose.pose.position.x = tfDesiredPose.getOrigin().x();
            // goal.target_pose.pose.position.y = tfDesiredPose.getOrigin().y();
            // goal.target_pose.pose.position.z = tfDesiredPose.getOrigin().z();
//
            // goal.target_pose.pose.orientation.w = tfDesiredPose.getRotation().w();
            // goal.target_pose.pose.orientation.x = tfDesiredPose.getRotation().x();
            // goal.target_pose.pose.orientation.y = tfDesiredPose.getRotation().y();
            // goal.target_pose.pose.orientation.z = tfDesiredPose.getRotation().z();

            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];

            goal.target_pose.pose.orientation.w = _home_rot[3];
            goal.target_pose.pose.orientation.x = _home_rot[0];
            goal.target_pose.pose.orientation.y = _home_rot[1];
            goal.target_pose.pose.orientation.z = _home_rot[2];

            ROS_INFO("Sending offset from Aruco as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("The mobile robot arrived 1 meter in front of Aruco");
            else{
                ROS_INFO("The base failed to move for some reason");
                return;
            }

            
            
            
            
            
            
            
            
        } else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
    
}

void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal_t( &TF_NAV::goal_listener, this );
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
    ros::spin();
}



int main( int argc, char** argv ) {
    ros::init(argc, argv, "tf_navigation");
    ros::NodeHandle nh;
    bool allowExploration;
    int numOfGoals;
    nh.getParam("allowExploration", allowExploration);
    nh.getParam("numberOfGoals", numOfGoals);
    TF_NAV tfnav(allowExploration, numOfGoals);
    tfnav.run();

    return 0;
}