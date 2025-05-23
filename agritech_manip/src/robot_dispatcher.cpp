#include <agritech_manip/robot_dispatcher.h>

namespace agritech_manip
{

    RobotDispatcher::RobotDispatcher(rclcpp::Node::SharedPtr node)
    {
        velocity_ = node->declare_parameter<double>("velocity", 0.8);
        acceleration_ = node->declare_parameter<double>("acceleration", 1.5);
        deceleration_ = node->declare_parameter<double>("deceleration", 2.0);

        try
        {
            // robot_planning_ = std::shared_ptr<ur10e_planning>(new
            // ur10e_planning(node));
            actionClientNode_ = std::shared_ptr<rclcpp::Node>(
                new rclcpp::Node("action_client_node"));
            robot_planning_ =
                std::shared_ptr<moveit_planning>(new moveit_planning(node));
            action_client_ = std::shared_ptr<simple_action_client_template>(
                new simple_action_client_template(actionClientNode_));
            node_ = node;
        }
        catch (std::exception &e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    RobotDispatcher::~RobotDispatcher() {}

    Eigen::Affine3d RobotDispatcher::getLinkFrameTransform(std::string &frame_id)
    {
        return robot_planning_->get_frame_transform(frame_id);
    }

    std::vector<double> RobotDispatcher::getJoints()
    {
        using namespace std::chrono_literals;

        // return robot_planning_->get_current_configuration();
        std::vector<double> jointsCurr(6);
        bool success;

        rclcpp::Client<ur_ros_rtde_msgs::srv::GetRobotConfiguration>::SharedPtr clientJoints =
            actionClientNode_->create_client<ur_ros_rtde_msgs::srv::GetRobotConfiguration>("/ur_ros_rtde/get_robot_configuration");

        while (!clientJoints->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return jointsCurr;
            }
            RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }

        auto request = std::make_shared<ur_ros_rtde_msgs::srv::GetRobotConfiguration::Request>();
        auto result_future = clientJoints->async_send_request(request);
        if (rclcpp::spin_until_future_complete(actionClientNode_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "call to service GetRobotConfiguration failed");
            return jointsCurr;
        }

        // if (result_future.valid)
        // {
            auto result = result_future.get();
            RCLCPP_INFO_STREAM(actionClientNode_->get_logger(), "received " << result->values.size() << " success " << result->success);
            if (result->success && result->values.size() == 6)
            {
                jointsCurr[0] = result->values[0];
                jointsCurr[1] = result->values[1];
                jointsCurr[2] = result->values[2];
                jointsCurr[3] = result->values[3];
                jointsCurr[4] = result->values[4];
                jointsCurr[5] = result->values[5];
            }
        // }

        return jointsCurr;
    }

    void RobotDispatcher::getWrench(geometry_msgs::msg::Wrench& wrench_value)
    {
        using namespace std::chrono_literals;

        // Initialize a client for the GetWrench service
        rclcpp::Client<ur_ros_rtde_msgs::srv::GetWrench>::SharedPtr clientWrench =
            actionClientNode_->create_client<ur_ros_rtde_msgs::srv::GetWrench>("/ur_ros_rtde/get_wrench");

        // Wait for the service to become available
        while (!clientWrench->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "Interrupted while waiting for the GetWrench service. Exiting.");
                return;
            }
            RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "Service GetWrench not available, waiting...");
        }

        // Create a request for the GetWrench service
        auto request = std::make_shared<ur_ros_rtde_msgs::srv::GetWrench::Request>();

        // Send the request asynchronously and wait for the result
        auto result_future = clientWrench->async_send_request(request);

        if (rclcpp::spin_until_future_complete(actionClientNode_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(actionClientNode_->get_logger(), "Call to GetWrench service failed");
            return;
        }

        // Get the result of the service call
        auto result = result_future.get();
        RCLCPP_INFO_STREAM(actionClientNode_->get_logger(), "Received wrench data, success: " << result->success);

        // If the call was successful, populate the wrench_value with the result
        if (result->success)
        {
            wrench_value.force.x = result->wrench.force.x;
            wrench_value.force.y = result->wrench.force.y;
            wrench_value.force.z = result->wrench.force.z;
            wrench_value.torque.x = result->wrench.torque.x;
            wrench_value.torque.y = result->wrench.torque.y;
            wrench_value.torque.z = result->wrench.torque.z;
        }
    }


    void RobotDispatcher::getJointState(sensor_msgs::msg::JointState &joints_value)
    {
        joints_value.position = getJoints();

        if (joints_value.position.size() != 6)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                                "invalid joint state position size "
                                    << joints_value.position.size()
                                    << ": it must be 6!");
        }
        joints_value.name.resize(6);
        joints_value.name[0] = "shoulder_pan_joint";
        joints_value.name[1] = "shoulder_lift_joint";
        joints_value.name[2] = "elbow_joint";
        joints_value.name[3] = "wrist_1_link";
        joints_value.name[4] = "wrist_2_link";
        joints_value.name[5] = "wrist_3_link";
        joints_value.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joints_value.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    bool RobotDispatcher::moveToPose(const Eigen::Affine3d &pose)
    {
        geometry_msgs::msg::Pose pose_msg;
        Eigen::Quaterniond q(pose.rotation());
        pose_msg.position.x = pose.translation()(0);
        pose_msg.position.y = pose.translation()(1);
        pose_msg.position.z = pose.translation()(2);
        pose_msg.orientation.w = q.w();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        return moveToPose(pose_msg);
    }

    bool RobotDispatcher::moveToPose(const geometry_msgs::msg::Pose &pose)
    {
        // trajectory_msgs::msg::JointTrajectory trajectory;
        std::vector<CandidatePlan> plans;
        std::vector<std::vector<double>> ik_solutions;
        CandidatePlan plan;

        std::cout << "Pose (x,y,z): ";
        std::cout << pose.position.x << " " << pose.position.y << " "
                  << pose.position.z << std::endl;
        std::cout << "Pose (x,y,z, w): ";
        std::cout << pose.orientation.x << " " << pose.orientation.y << " "
                  << pose.orientation.z << " " << pose.orientation.w << std::endl;

        robot_planning_->inverse_kinematics(pose, ik_solutions);

        // std::vector<double> current_joint_state =
        //     robot_planning_->get_current_joint_values();
        std::vector<double> current_joint_state =
            robot_planning_->get_current_configuration();

        std::cout << "current state: ";
        for (auto jj : current_joint_state)
            std::cout << jj << " ";
        std::cout << std::endl;

        int indexMin = -1; // index of the best pose
        double scoreMin = INFINITY;

        // Plans a trajectory for each solution of inverse kinemantics and
        // scores the solution
        plans.reserve(ik_solutions.size());
        for (size_t i = 0; i < ik_solutions.size(); ++i)
        {
            plan.joints = ik_solutions[i];
            plan.success = robot_planning_->plan_trajectory(
                ik_solutions[i], plan.trajectory, plan.orientationTolerances);
            if (plan.success)
            {
                plan.score = getTrajectoryCost(plan.trajectory);
                if (indexMin < 0 || plan.score < scoreMin)
                {
                    indexMin = (int)i;
                    scoreMin = plan.score;
                }
            }
            plans.push_back(plan);
        }
        if (0 <= indexMin && indexMin < plans.size())
        {
            std::cout << "Solution found, best score: " << plans[indexMin].score
                      << std::endl;

            auto goal_msg = ur_ros_rtde_msgs::action::ExecuteTrajectory::Goal();
            goal_msg.speed = velocity_;
            goal_msg.acceleration = acceleration_;
            goal_msg.deceleration = deceleration_;
            goal_msg.trajectory
                .clear(); // resize(plans[indexMin].trajectory.points.size());

            // Copies the joint values vector for each single point of the
            // planned trajectory (index i) from
            for (size_t i = 0; i < plans[indexMin].trajectory.points.size(); ++i)
            {
                ur_ros_rtde_msgs::msg::Vector joint_vector;
                joint_vector.vector.resize(6);
                for (int jj = 0; jj < 6; ++jj)
                {
                    joint_vector.vector[jj] = plans[indexMin].trajectory.points[i].positions[jj];
                }
                // joint_vector.vector = {
                //     plans[indexMin].trajectory.points[i].positions[0],
                //     plans[indexMin].trajectory.points[i].positions[1],
                //     plans[indexMin].trajectory.points[i].positions[2],
                //     plans[indexMin].trajectory.points[i].positions[3],
                //     plans[indexMin].trajectory.points[i].positions[4],
                //     plans[indexMin].trajectory.points[i].positions[5]};
                std::cout << i << "/" << plans[indexMin].trajectory.points.size() << ") ";
                for (int jj = 0; jj < 6; jj++)
                {
                    std::cout << joint_vector.vector[jj] << " ";
                }
                goal_msg.trajectory.push_back(joint_vector);
                std::cout << std::endl;
            }
            std::cout << __FILE__ << "," << __LINE__ << ": execute planned trajectory" << std::endl;

            action_client_->send_goal<ur_ros_rtde_msgs::action::ExecuteTrajectory>(
                "ur_ros_rtde/execute_trajectory_command", goal_msg);
            return true;
        }
        else
        {
            std::cerr << __FILE__ << "," << __LINE__ << ": FAILED Planning"
                      << std::endl;
            return false;
        }
    }

    bool RobotDispatcher::moveToJoints(const sensor_msgs::msg::JointState &joints)
    {
        // trajectory_msgs::msg::JointTrajectory trajectory;
        // std::vector<double> orientationTolerances;
        // robot_planning_.plan_trajectory(joints.position, trajectory,
        //                                 orientationTolerances);

        CandidatePlan plan;
        plan.joints = joints.position;
        std::cout << "planning to: ";
        for (auto jj : plan.joints)
            std::cout << jj << " ";
        std::cout << std::endl;
        plan.success = robot_planning_->plan_trajectory(
            plan.joints, plan.trajectory, plan.orientationTolerances);
        if (plan.success)
        {
            auto goal_msg = ur_ros_rtde_msgs::action::ExecuteTrajectory::Goal();
            goal_msg.speed = velocity_;
            goal_msg.acceleration = acceleration_;
            goal_msg.deceleration = deceleration_;
            goal_msg.trajectory.clear();

            for (size_t i = 0; i < plan.trajectory.points.size(); ++i)
            {
                ur_ros_rtde_msgs::msg::Vector joint_vector;
                joint_vector.vector = {plan.trajectory.points[i].positions[0],
                                       plan.trajectory.points[i].positions[1],
                                       plan.trajectory.points[i].positions[2],
                                       plan.trajectory.points[i].positions[3],
                                       plan.trajectory.points[i].positions[4],
                                       plan.trajectory.points[i].positions[5]};
                for (int jj = 0; jj < 6; jj++)
                {
                    std::cout << joint_vector.vector[jj] << " ";
                }
                goal_msg.trajectory.push_back(joint_vector);
                std::cout << std::endl;
            }

            // goal_msg.trajectory.resize(plan.trajectory.points.size());

            // // Copies the joint values vector for each single point of the
            // // planned trajectory (index i) from
            // for (size_t i = 0; i < plan.joints.size(); ++i)
            // {
            //     goal_msg.trajectory[i].vector =
            //     plan.trajectory.points[i].positions;
            // }

            action_client_->send_goal<ur_ros_rtde_msgs::action::ExecuteTrajectory>(
                "ur_ros_rtde/execute_trajectory_command", goal_msg);

            std::cout << "trajectory sent" << std::endl;
            return true;
        }
        return false;
    }

    bool RobotDispatcher::moveToJointsAsync(const sensor_msgs::msg::JointState &joints)
    {
        // trajectory_msgs::msg::JointTrajectory trajectory;
        // std::vector<double> orientationTolerances;
        // robot_planning_.plan_trajectory(joints.position, trajectory,
        //                                 orientationTolerances);

        CandidatePlan plan;
        plan.joints = joints.position;
        std::cout << "planning to: ";
        for (auto jj : plan.joints)
            std::cout << jj << " ";
        std::cout << std::endl;
        plan.success = robot_planning_->plan_trajectory(
            plan.joints, plan.trajectory, plan.orientationTolerances);
        if (plan.success)
        {
            auto goal_msg = ur_ros_rtde_msgs::action::ExecuteTrajectory::Goal();
            goal_msg.speed = velocity_;
            goal_msg.acceleration = acceleration_;
            goal_msg.deceleration = deceleration_;
            goal_msg.trajectory.clear();

            for (size_t i = 0; i < plan.trajectory.points.size(); ++i)
            {
                ur_ros_rtde_msgs::msg::Vector joint_vector;
                joint_vector.vector = {plan.trajectory.points[i].positions[0],
                                       plan.trajectory.points[i].positions[1],
                                       plan.trajectory.points[i].positions[2],
                                       plan.trajectory.points[i].positions[3],
                                       plan.trajectory.points[i].positions[4],
                                       plan.trajectory.points[i].positions[5]};
                for (int jj = 0; jj < 6; jj++)
                {
                    std::cout << joint_vector.vector[jj] << " ";
                }
                goal_msg.trajectory.push_back(joint_vector);
                std::cout << std::endl;
            }

            // goal_msg.trajectory.resize(plan.trajectory.points.size());

            // // Copies the joint values vector for each single point of the
            // // planned trajectory (index i) from
            // for (size_t i = 0; i < plan.joints.size(); ++i)
            // {
            //     goal_msg.trajectory[i].vector =
            //     plan.trajectory.points[i].positions;
            // }

            simple_action_client_template::async_goal_state<ur_ros_rtde_msgs::action::ExecuteTrajectory> goal_state;
            action_client_->send_goal_async<ur_ros_rtde_msgs::action::ExecuteTrajectory>(
                "ur_ros_rtde/execute_trajectory_command", goal_msg, goal_state_execute_trajectory_);

            std::cout << "trajectory sent" << std::endl;
            return true;
        }
        return false;
    }

    bool RobotDispatcher::is_trajectory_completed()
    {
        const std::string action_server_name = "ur_ros_rtde/execute_trajectory_command"; // The action server name for trajectory execution

        // Call the get_async_action_status function, passing in the action server name and goal_state_execute_trajectory_
        bool result = action_client_->get_async_action_status<ur_ros_rtde_msgs::action::ExecuteTrajectory>(action_server_name, goal_state_execute_trajectory_);

        if (result)
        {
            std::cout << "Trajectory execution completed successfully." << std::endl;
            return true;
        }
        else
        {
            std::cout << "Trajectory execution did not succeed or is not yet completed." << std::endl;
            return false;
        }

        
    }

    void RobotDispatcher::cancel_action()
    {
        // Use the action server name and goal state, just like in get_async_action_status
        const std::string action_server_name = "ur_ros_rtde/execute_trajectory_command";

        // Call the cancel_async_goal function, passing in the action server name and goal_state_execute_trajectory_
        bool result = action_client_->cancel_async_goal<ur_ros_rtde_msgs::action::ExecuteTrajectory>(action_server_name, goal_state_execute_trajectory_);

        if (result)
        {
            RCLCPP_INFO(node_->get_logger(), "Trajectory execution successfully cancelled.");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to cancel trajectory execution.");
        }
    }



    bool RobotDispatcher::moveUntilTorque(const Eigen::Vector3d &position, double torqueThres)
    {
        auto goal_msg =
            ur_ros_rtde_msgs::action::MoveUntilTorque::Goal();

        return true;
    }

    bool RobotDispatcher::moveUntilContact(const Eigen::Vector3d &toolSpeed, const Eigen::Vector3d &direction, double acceleration)
    {
        auto goal_msg =
            ur_ros_rtde_msgs::action::MoveUntilContact::Goal();
        goal_msg.toolspeed.push_back(toolSpeed(0));
        goal_msg.toolspeed.push_back(toolSpeed(1));
        goal_msg.toolspeed.push_back(toolSpeed(2));
        goal_msg.direction.push_back(direction(0));
        goal_msg.direction.push_back(direction(1));
        goal_msg.direction.push_back(direction(2));
        goal_msg.acceleration = acceleration;
        return true;
    }

    void RobotDispatcher::commandGripper(int grip_witdh)
    {
        auto goal_msg =
            ur_ros_rtde_gripper_commands::action::SoftGripperControl::Goal();
        goal_msg.grip = true;
        goal_msg.target_width = grip_witdh;
        action_client_
            ->send_goal<ur_ros_rtde_gripper_commands::action::SoftGripperControl>(
                "ur_ros_rtde/soft_gripper_control_command", goal_msg);
    }

    double RobotDispatcher::getTrajectoryCost(
        const trajectory_msgs::msg::JointTrajectory &trajectory)
    {
        double sum, dd;
        sum = 0.0;
        for (size_t j = 1; j < trajectory.points.size(); ++j)
        {
            dd = 0.0;
            for (size_t k = 0; k < 6; ++k)
            {
                dd += std::fabs(trajectory.points[j].positions[k] -
                                trajectory.points[j - 1].positions[k]);
            }
            sum += dd;
        }
        return sum;
    }

} // end of namespace agritech_manip