#ifndef TOMATO_PICK_H_
#define TOMATO_PICK_H_

#include <Eigen/Dense>
#include <iostream>

// General ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

// Messages
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tomato_msg/msg/tomato_msg.hpp>

#include <geometry_msgs/msg/wrench.hpp> 

// Interfaces
// #include <ur10e_planning/ur10e_planning.h>
#include <agritech_manip/robot_dispatcher.h>
#include <agritech_manip/static_object_tracker.h>
#include <agritech_manip/transform.h>


#include <cmath>


namespace agritech_manip
{

    class TomatoPickNode
    {
    public:
        using Object = typename StaticObjectTracker::Object;
        using VectorObject = typename StaticObjectTracker::VectorObject;

        enum class State
        {
            INIT,
            SCENE_OBSERVATION,
            CLOSE_OBSERVATION,
            APPROACH,
            PICK,
            PLACE,
            DUMMY_INIT,
            DUMMY_HOME,
            DUMMY_DEPOSIT,
            DUMMY_WRENCH,
            DUMMY_ROTATE,
            DUMMY_END
        };

        /**
         * @brief Constructor of a new node object to control the pinking of
         * tomatoes.
         *
         * @param nodeName the name of the node
         */
        TomatoPickNode(const std::string &nodeName);

        /**
         * @brief Destructor.
         */
        ~TomatoPickNode();

        /**
         * @brief Callback function associated to publising of a message with the
         * list of candidate tomatoes.
         *
         * @param msg the message with the candidates of tomatoes
         */
        void onTomatoDetected(const tomato_msg::msg::TomatoMsg &msg);

        /**
         * @brief Function with the main loop.
         *
         */
        void onLoopExecution();

        /**
         * @brief Spins the internal node to capture events.
         *
         */
        void spin();

        /**
         * @brief Returns the name string of a given state.
         */
        static std::string getStateName(const State &state);

    private:
        // Members related to ROS framework
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<tomato_msg::msg::TomatoMsg>::SharedPtr tomatoSub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // Members related to agritech_manip system
        StaticObjectTracker tracker_;
        VectorObject objectsCamera_;
        std::shared_ptr<RobotDispatcher> robot_;
        // Members related to the task
        State state_;
        std::string world_frame_id_;
        std::string wrist_frame_id_;
        std::string camera_frame_id_;
        std::string gripper_frame_id_;
        Eigen::Affine3d transform_wrist_camera_;
        Eigen::Affine3d transform_wrist_camera_curr_;
        sensor_msgs::msg::JointState joints_home_;
        sensor_msgs::msg::JointState joints_deposit_;
        bool enableTracking_;
        Eigen::Vector3d targetPosition_;
        Eigen::Vector3d approach_dir_;
        double approach_dir_pitch1_;
        double approach_dir_pitch2_;
        Eigen::Vector3d place_position_;
        Eigen::Vector3d place_dir_;
        Eigen::Affine3d transform_world_wrist_goal_;
        double observation_distance_;
        double grasp_distance_close_;
        double grasp_distance_far_;
        double place_distance_;
        int gripper_width_open_;
        int gripper_width_closed_;

        Eigen::Affine3d readTransform(const std::string &position_param,
                                      const std::vector<double> &position_default,
                                      const std::string &rot_rpy_param,
                                      const std::vector<double> &rot_rpy_default);

        sensor_msgs::msg::JointState readJointState(
            const std::string &joint_param,
            const std::vector<double> &joint_default);

        void readVector3(
            const std::string &vec_param,
            Eigen::Vector3d &vec_value);

        void selectTargetObject(Object& target, const VectorObject& objects);

        void computeLoSPose(Eigen::Affine3d &poseWristTarget);

        void computeGraspPoseAdaptable(Eigen::Affine3d &poseWristTarget, double distance, double approach_dir_pitch);

        void computeGraspPose(Eigen::Affine3d &poseWristTarget, double distance);

        void publishTf();

        void publishTransform(const Eigen::Affine3d &pose,
                              const std::string &frame_id,
                              const std::string &child_frame_id,
                              const builtin_interfaces::msg::Time &stamp);

        void publishPosition(const Eigen::Vector3d &position,
                             const std::string &frame_id,
                             const std::string &child_frame_id,
                             const builtin_interfaces::msg::Time &stamp);
    };

} // end of namespace agritech_manip

#endif