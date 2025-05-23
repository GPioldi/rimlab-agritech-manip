// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// #include <agritech_manip/action_client_template.h>
// #include <ur10e_planning/ur10e_planning.h>
#include <moveit_planning/moveit_planning.hpp>
#include <ur_ros_rtde_gripper_commands/action/soft_gripper_control.hpp>
#include <ur_ros_rtde_msgs/action/execute_trajectory.hpp>
#include <ur_ros_rtde_msgs/action/move_until_torque.hpp>
#include <ur_ros_rtde_msgs/action/move_until_contact.hpp>
#include <ur_ros_rtde_msgs/msg/vector.hpp>
#include <ur_ros_rtde_msgs/srv/get_robot_configuration.hpp>


#include <ur_ros_rtde_msgs/srv/get_wrench.hpp>
//#include "ur_ros_rtde_msgs/action/execute_trajectory.hpp" 


#include <ur_ros_rtde_simple_clients/simple_action_client_template.hpp>

namespace agritech_manip {

/**
 * @brief class RobotDispatcher encapsulates the elementary operations
 *  related to robot planning.
 *
 */
class RobotDispatcher {
   public:
    /**
     * @brief Structure with the data corresponding to a plan.
     *
     */
    struct CandidatePlan {
        std::vector<double> joints;
        trajectory_msgs::msg::JointTrajectory trajectory;
        std::vector<double> orientationTolerances;
        bool success;
        double score;
    };

    /**
     * @brief Default constructor.
     *
     */
    RobotDispatcher(rclcpp::Node::SharedPtr node);

    /**
     * @brief Destructor.
     *
     */
    ~RobotDispatcher();

    /**
     * @brief Gets the pose of the given frame frame_id w.r.t. the planning
     * scene reference frame.
     *
     * @param frame_id
     * @return Eigen::Affine3d
     */
    Eigen::Affine3d getLinkFrameTransform(std::string& frame_id);

    std::vector<double> getJoints();

    /**
     * 
     */
    void getJointState(sensor_msgs::msg::JointState& joints_value);

    /**
     * @brief Gets the wrench (force and torque) data from the robot.
     *
     * This function requests the wrench (force and torque) data from the robot by calling 
     * the /ur_ros_rtde/get_wrench service. It populates the provided geometry_msgs::msg::Wrench 
     * object with the retrieved force and torque values if the service call is successful.
     *
     * @param wrench_value Reference to a geometry_msgs::msg::Wrench object where the 
     *                     received wrench data will be stored. This includes force (x, y, z) 
     *                     and torque (x, y, z).
     */
    void getWrench(geometry_msgs::msg::Wrench& wrench_value); 

    /**
     * @brief Checks whether the trajectory execution has been completed successfully.
     * 
     * This function calls the asynchronous action server to determine if the trajectory
     * execution action has finished successfully. It uses the stored goal state from the
     * action client to fetch the result of the action.
     * 
     * @return True if the trajectory execution was successful, false otherwise.
     */
    bool is_trajectory_completed();

    /**
     * @brief Cancels the current trajectory execution.
     *
     * This function attempts to cancel an ongoing trajectory execution by 
     * sending a cancellation request to the action server. It uses the
     * goal state `goal_state_execute_trajectory_` to specify the current 
     * action being executed. The action server name is 
     * "ur_ros_rtde/execute_trajectory_command".
     *
     * No return value is provided, but the result of the cancellation 
     * is logged to indicate whether the action was successfully canceled or not.
     */
    void cancel_action();


    /**
     * @brief Plans and executes the motion from current configuration
     * to the desired pose of the wrist frame of the robot.
     * It solves inverse kinematics and selects the path that requires
     * reduced changes in the joint values.
     *
     * @param pose the target wrist pose provided as Eigen::Affine3d
     */
    bool moveToPose(const Eigen::Affine3d& pose);

    /**
     * @brief Plans and executes the motion from current configuration
     * to the desired pose of the wrist frame of the robot.
     * It solves inverse kinematics and selects the path that requires
     * reduced changes in the joint values.
     *
     * @param pose the target wrist pose provided as ROS Pose message
     */
    bool moveToPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Plans and executes the path moving the robot to the given
     * joint configuration of the robot.
     *
     * @param joints
     */
    bool moveToJoints(const sensor_msgs::msg::JointState& joints);

    /**
     * @brief Plans and executes the path moving the robot to the given
     * joint configuration of the robot in asynchronus way.
     *
     * @param joints
     */
    bool moveToJointsAsync(const sensor_msgs::msg::JointState& joints);



    /**
     * @brief Moves the robot towards a given position untile torque is larger 
     * than given threshold.
     *
     * @param joints
     */
    bool moveUntilTorque(const Eigen::Vector3d& position, double torqueThres);

    bool moveUntilContact(const Eigen::Vector3d& toolSpeed, const Eigen::Vector3d& direction, double acceleration);

    /**
     * @brief Opens the soft gripper to the desided width, i.e. the
     * opening diameter of the gripper.
     *
     * @param grip_witdh
     */
    void commandGripper(int grip_witdh);

   private:
    // std::shared_ptr<ur10e_planning> robot_planning_;
    std::shared_ptr<moveit_planning> robot_planning_;
    std::shared_ptr<simple_action_client_template> action_client_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr actionClientNode_;
    double velocity_;
    double acceleration_;
    double deceleration_;

    // Define goal_state as a private member to be used in the class
    simple_action_client_template::async_goal_state<ur_ros_rtde_msgs::action::ExecuteTrajectory> goal_state_execute_trajectory_;

    double getTrajectoryCost(
        const trajectory_msgs::msg::JointTrajectory& trajectory);
};

}  // namespace agritech_manip
