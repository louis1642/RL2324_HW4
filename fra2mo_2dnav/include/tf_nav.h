#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <array>
#include <vector>

#define NUM_GOALS 4

std::vector<double> aruco_pose(7,0.0);
//tf::Transform tfAruco;

//void arucoPoseCallback(const geometry_msgs::PoseStamped & msg);

class TF_NAV {

    public:
        TF_NAV(bool allowExploration, int totalNumberOfGoals);
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();
        void arucoPoseCallback(const geometry_msgs::PoseStamped & msg);


private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;
        ros::Subscriber _aruco_pose_sub;

        Eigen::Vector3d _home_pos;
        Eigen::Vector4d _home_rot;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

        tf::Transform _tfBaseCamera;    // transformation matrix from camera to base
        tf::Transform _tfAruco;         // pose of the Aruco Marker in base frame
        tf::Transform _tfBase;          // current pose of the base footprint
        bool _allowExploration;
        const int _totalNumberOfGoals;

        Eigen::Vector3d _aruco_goal_pos;
        Eigen::Vector4d _aruco_goal_or;
        std::vector<Eigen::Vector3d> _goal_pos;
        std::vector<Eigen::Vector4d> _goal_or;

        int goalOrder[NUM_GOALS] = {3, 4, 2, 1};


        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


};