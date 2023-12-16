#include "../include/tf_nav.h"

void TF_NAV::arucoPoseCallback(const geometry_msgs::PoseStamped & msg) {

    tf::Vector3 ArucoPosition(
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Quaternion ArucoOrientation(
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Transform tfCameraAruco = tf::Transform(ArucoOrientation, ArucoPosition);


    // aruco wrt world frame
    _tfAruco = _tfBase * _tfBaseCamera * tfCameraAruco;

    static tf::TransformBroadcaster arucoTfBroadcaster;
    // the broadcaster sends a stamped transform:
    //  time, frame_id and child_frame_id are to be added to the transform
    arucoTfBroadcaster.sendTransform(
            tf::StampedTransform(_tfAruco, ros::Time::now(), "map", "tf_marker_frame"));

}

TF_NAV::TF_NAV(bool allowExploration, const int totalNumberOfGoals)
        : _totalNumberOfGoals(totalNumberOfGoals) {
    // member initializer list for _totalNumberOfGoals as it is const

    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    // subscribe to the aruco single pose topic
    _aruco_pose_sub = _nh.subscribe("/aruco_single/pose", 1, &TF_NAV::arucoPoseCallback, this);

    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    // resizing the std::vectors
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
        listener.waitForTransform( "base_footprint", "camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform( "base_footprint", "camera_depth_optical_frame", ros::Time(0), tfBaseCamera );
    } catch ( tf::TransformException &ex ) {
        ROS_ERROR("%s", ex.what());
        r.sleep();
        return;
    }

    // casting a StampedTransform to a Transform (the header is not needed)
    _tfBaseCamera = tfBaseCamera;
}

void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while ( ros::ok() ) {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );
        } catch ( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            // if it fails, skipt to the next iteration
            continue;
        }

        _tfBase = transform;    // store the tf matrix of base_footprint into object data
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
    tf::TransformListener listener;
    // transform is defined as vector of pointers in order to use std::vector::resize()
    std::vector<tf::StampedTransform*> transform;
    tf::StampedTransform tfGoalAruco;
    transform.resize(_totalNumberOfGoals);
    static std::vector<bool> hasLogged;
    hasLogged.resize(_totalNumberOfGoals);
    for (int i = 0; i < _totalNumberOfGoals; ++i) {
        // initialization of the vectors
        transform[i] = new tf::StampedTransform();
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
                    // print info about the retrieved transform
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

                // store position and orientation of the targets in the vectors
                _goal_pos.at(goal_number) << transform[goal_number]->getOrigin().x(), transform[goal_number]->getOrigin().y(), transform[goal_number]->getOrigin().z();
                _goal_or.at(goal_number) << transform[goal_number]->getRotation().w(),  transform[goal_number]->getRotation().x(), transform[goal_number]->getRotation().y(), transform[goal_number]->getRotation().z();

            } catch ( tf::TransformException &ex ) {
                ROS_ERROR("goal_number = %d: %s", goal_number, ex.what());
                r.sleep();
                // skip to the next iteration
                continue;
            }
        }

        // acquire the goal to move the robot near the aruco marker
        try {
            listener.waitForTransform( "map", "goal_marker_frame", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform("map", "goal_marker_frame", ros::Time( 0 ),
                                     tfGoalAruco);

            _aruco_goal_pos << tfGoalAruco.getOrigin().x(), tfGoalAruco.getOrigin().y(), tfGoalAruco.getOrigin().z();
            _aruco_goal_or << tfGoalAruco.getRotation().w(),  tfGoalAruco.getRotation().x(), tfGoalAruco.getRotation().y(), tfGoalAruco.getRotation().z();

        } catch ( tf::TransformException &ex ) {
            ROS_ERROR("goal_aruco %s", ex.what());
            r.sleep();
            // skips to the next iteration of the while (ros::ok()) loop
            continue;
        }

        r.sleep();
    }
}

void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;

    while ( ros::ok() ) {
        // prompt the user for the command
        std::cout<<"\nInsert 1 to send goal from TF ";
        if (_allowExploration) {
            // distinguish between the 4 goals in the hw and the n goals for exploration
            std::cout << "for exploration";
        }
        std::cout << "\nInsert 2 to send home position goal ";
        std::cout << "\nInsert 3 to send aruco marker goal " << std::endl;
        std::cout << "Insert your choice" << std::endl;
        std::cin >> cmd;

        if ( cmd == 1) {        // goals from tf
            MoveBaseClient ac("move_base", true);
            while (!ac.waitForServer(ros::Duration(5.0))) {
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
                // wait 0.1 seconds to prevent a racing condition btw ac.sendGoal() and ac.waitForResult()
                ros::Duration(0.1).sleep();
                ac.sendGoal(goal);
//                ros::Duration(1).sleep();
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    // conditional operator to print the correct message
                    ROS_INFO("The mobile robot arrived in the TF goal number %d", (!_allowExploration) ? goalOrder[goal_index] : goal_index);
                } else {
                    ROS_INFO("The base failed to move for some reason");
                    // skip to the next iteration (skip to the next goal)
                    // or retry the same goal?
                    // --goal_index;
                    // best would be to send another goal to try to get the robot unstuck.. too long to implement
                    continue;
                }
            }
        } else if ( cmd == 2 ) {        // return home
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

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The mobile robot arrived in the HOME position");
            } else {
                ROS_INFO("The base failed to move for some reason");
            }
        } else if (cmd == 3) {      // visual task
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

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The mobile robot arrived in the Aruco near position");
            } else {
                ROS_INFO("The base failed to move for some reason");
                return;
            }
            // At this point, the Aruco pose is available and the robot has to move 1 meter in front of it
            //tf::Matrix3x3 rotOffset(0,-1,0, 0,0,1,  -1,0,0);
            tf::Quaternion rotOffset(-0.5, 0.5, 0.5, 0.5);
            tf::Vector3 posOffset(0,-_tfBaseCamera.getOrigin().z(),1);
            tf::Transform tfOffset(rotOffset,posOffset);

            ros::Duration(2.0).sleep();
            tf::Transform tfDesiredPose = _tfAruco*tfOffset;

            Eigen::Vector3d des_pos;
            Eigen::Vector4d des_or;
            des_pos << tfDesiredPose.getOrigin().x(), tfDesiredPose.getOrigin().y(), tfDesiredPose.getOrigin().z();
            des_or << tfDesiredPose.getRotation().w(),  tfDesiredPose.getRotation().x(), tfDesiredPose.getRotation().y(), tfDesiredPose.getRotation().z();
            std::cout << "desired orientation:\n" << des_or << std::endl;
            std::cout << "aruco goal orientation:\n" << _aruco_goal_or << std::endl;


            // broadcast the tf to visualize it in RViz
            static tf::TransformBroadcaster arucoTfBroadcaster;
            arucoTfBroadcaster.sendTransform(tf::StampedTransform(tfDesiredPose, ros::Time::now(), "map", "offset_goal_frame"));

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = tfDesiredPose.getOrigin().x();
            goal.target_pose.pose.position.y = tfDesiredPose.getOrigin().y();
            goal.target_pose.pose.position.z = 0.1;

            // round to first decimal digit. This is done to avoid the robot thinking
            //  a certain trajectory is unfeasible because of numerical error
            goal.target_pose.pose.orientation.w = std::round(tfDesiredPose.getRotation().w()*10)/10;
            goal.target_pose.pose.orientation.x = std::round(tfDesiredPose.getRotation().x()*10)/10;
            goal.target_pose.pose.orientation.y = std::round(tfDesiredPose.getRotation().y()*10)/10;
            goal.target_pose.pose.orientation.z = std::round(tfDesiredPose.getRotation().z()*10)/10;

            ROS_INFO("Sending offset from Aruco as goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The mobile robot arrived 1 meter in front of Aruco");
                return;
            } else {
                ROS_INFO("The base failed to move for some reason");
            }

        } else {        // neither 1, 2 nor 3 input as command
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
    // retrieve the parameters from Ros parameters server
    nh.getParam("allowExploration", allowExploration);
    nh.getParam("numberOfGoals", numOfGoals);
    TF_NAV tfnav(allowExploration, numOfGoals);
    tfnav.run();

    return 0;
}