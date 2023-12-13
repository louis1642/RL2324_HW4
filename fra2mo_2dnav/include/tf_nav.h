#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <array>
#include <vector>

#define NUM_GOALS 4

class TF_NAV {

    public:
        explicit TF_NAV(bool allowExploration);
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal_listener();
        void send_goal();

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;
        Eigen::Vector4d _home_rot;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;

//        Eigen::Vector3d _goal_pos;
//        Eigen::Vector4d _goal_or;
//        std::array<Eigen::Vector3d, NUM_GOALS> _goal_pos;
//        std::array<Eigen::Vector4d, NUM_GOALS> _goal_or;
        std::vector<Eigen::Vector3d> _goal_pos;
        std::vector<Eigen::Vector4d> _goal_or;

        int goalOrder[NUM_GOALS] = {3, 4, 2, 1};

        bool _allowExploration;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


};