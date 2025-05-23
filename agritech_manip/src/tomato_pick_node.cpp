// #include "agritech_manip/TomatoPickNode.h"
#include "agritech_manip/tomato_pick_node.h"

namespace agritech_manip
{

    TomatoPickNode::TomatoPickNode(const std::string &nodeName)
        : node_(new rclcpp::Node(nodeName))
    {
        std::string tomato_position_topic;
        int cycleTimeMs;
        double association_distance;
        int hit_num_min, miss_num_max;

        tomato_position_topic = node_->declare_parameter<std::string>(
            "tomato_position_topic", "/tomato_position");
        cycleTimeMs = node_->declare_parameter("cycle_time_ms", 50);
        world_frame_id_ =
            node_->declare_parameter<std::string>("world_frame_id", "world");
        wrist_frame_id_ =
            node_->declare_parameter<std::string>("wrist_frame_id", "wrist_3_link");
        camera_frame_id_ = node_->declare_parameter<std::string>(
            "camera_frame_id", "camera_color_optical_frame");
        gripper_frame_id_ = node_->declare_parameter<std::string>(
            "gripper_frame_id", "soft_gripper");

        // Reads the fixed orientation of camera frame w.r.t. the wrist frame
        std::vector<double> position_wrist_camera_default(
            {0.009, -0.08245, 0.09533});
        std::vector<double> rot_rpy_wrist_camera_default({180.0, 0.0, 0.0});
        transform_wrist_camera_ =
            readTransform("position_wrist_camera", position_wrist_camera_default,
                          "rot_rpy_wrist_camera", rot_rpy_wrist_camera_default);
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "transform_wrist_camera_:\n"
                               << transform_wrist_camera_.matrix());
        // Reads the Home configuration of robot
        std::vector<double> joints_home_default(
            {1.57, -2.036, 2.22, -0.14, 1.57, 3.14});
        for (auto &val : joints_home_default)
        {
            std::cout << "joints_home_default " << val << std::endl;
        }
        joints_home_ = readJointState("joints_home", joints_home_default);
        // Reads the Deposit configuration of robot
        std::vector<double> joints_deposit_default(
            {3.2059, -1.9259, 2.2249, -0.3885, 1.8129, 3.2097});
        for (auto &val : joints_home_default)
        {
            std::cout << "joints_deposit_default " << val << std::endl;
        }
        joints_deposit_ = readJointState("joints_deposit", joints_deposit_default);
        // Parameters related to poses required for task execution
        observation_distance_ =
            node_->declare_parameter<double>("observation_distance", 0.30);
        grasp_distance_far_ =
            node_->declare_parameter<double>("grasp_distance_far", 0.23);
        grasp_distance_close_ =
            node_->declare_parameter<double>("grasp_distance_close", 0.15);
        place_distance_ =
            node_->declare_parameter<double>("place_distance", 0.40);
        sensor_msgs::msg::JointState joints_value;

        // std::vector<double> approach_dir_vec = node_->declare_parameter<std::vector<double>>(
        //     "approach_dir", {0.0, 0.0, 1.0});
        // if (approach_dir_vec.size() == 3)
        // {
        //     approach_dir_ << approach_dir_vec[0], approach_dir_vec[1], approach_dir_vec[2];
        //     approach_dir_.normalize();
        // }
        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //                    "approach_dir_:\n"
        //                        << approach_dir_.transpose());
        readVector3("approach_dir", approach_dir_);

        approach_dir_pitch1_ = node_->declare_parameter<double>("approach_dir_pitch1_deg", 10.0);
        approach_dir_pitch1_ = M_PI / 180.0 * approach_dir_pitch1_;
        approach_dir_pitch2_ = node_->declare_parameter<double>("approach_dir_pitch2_deg", 20.0);
        approach_dir_pitch2_ = M_PI / 180.0 * approach_dir_pitch2_;

        // std::vector<double> place_dir_vec = node_->declare_parameter<std::vector<double>>(
        //     "place_dir", {0.0, 0.0, 1.0});
        // if (place_dir_vec.size() == 3)
        // {
        //     place_dir_ << place_dir_vec[0], place_dir_vec[1], place_dir_vec[2];
        //     place_dir_.normalize();
        // }
        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //                    "place_dir_:\n"
        //                        << approach_dir_.transpose());
        // std::vector<double> place_dir_vec = node_->declare_parameter<std::vector<double>>(
        //     "place_dir", {0.0, 0.0, 1.0});
        // if (place_dir_vec.size() == 3)
        // {
        //     place_dir_ << place_dir_vec[0], place_dir_vec[1], place_dir_vec[2];
        //     place_dir_.normalize();
        // }
        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //                    "place_dir_:\n"
        //                        << approach_dir_.transpose());
        readVector3("place_dir", place_dir_);
        readVector3("place_position", place_position_);

        association_distance =
            node_->declare_parameter<double>("association_distance", 0.10);
        hit_num_min = node_->declare_parameter<int>("hit_num_min", 6);
        miss_num_max = node_->declare_parameter<int>("miss_num_max", 10);
        tracker_.setAssociationDistance(association_distance);
        tracker_.setHitMissBounds(hit_num_min, miss_num_max);

        gripper_width_open_ = node_->declare_parameter<int>("gripper_width_open", 74);
        gripper_width_closed_ = node_->declare_parameter<int>("gripper_width_closed", 30);

        try
        {
            // robot_ = std::shared_ptr<RobotDispatcher>(new
            // RobotDispatcher(shared_from_this()));
            robot_ = std::shared_ptr<RobotDispatcher>(new RobotDispatcher(node_));
        }
        catch (std::exception &e)
        {
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                "Error in initialization of robot dispatcher: " << e.what());
        }

        tomatoSub_ = node_->create_subscription<tomato_msg::msg::TomatoMsg>(
            tomato_position_topic, 10,
            std::bind(&TomatoPickNode::onTomatoDetected, this, _1));

        timer_ = node_->create_wall_timer(
            std::chrono::duration<int, std::milli>(cycleTimeMs),
            std::bind(&TomatoPickNode::onLoopExecution, this));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO_STREAM(node_->get_logger(), "Initialized node with topic \""
                                                    << tomato_position_topic
                                                    << "\"");

        // Initial values:
        // * state_ is set to init
        // * enableTracking_ is false since the robot initially moves to home configuration
        // * targetPosition_ is set in world frame just to give a meaningful value
        // *
        
        //state_ = State::INIT;
        state_ = State::DUMMY_INIT;
        //joints_home_ = {-0.0026, -0.862, -2.2731, 0.0068, 1.5067, 3.0997};
        //joints_deposit_ = {0.03, -1.9647, -1.83, 0.7389, 1.48, 3.0995};

        enableTracking_ = false;
        targetPosition_ = robot_->getLinkFrameTransform(world_frame_id_).translation();
        transform_world_wrist_goal_ = robot_->getLinkFrameTransform(wrist_frame_id_);
    }

    TomatoPickNode::~TomatoPickNode() {}

    void TomatoPickNode::onTomatoDetected(const tomato_msg::msg::TomatoMsg &msg)
    {
        VectorObject objetsWorld;
        Object object;

        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "tomato message received: reference frame \""
                               << msg.header.frame_id << "\"");

        // geometry_msgs::msg::TransformStamped tfWorldCameraRos, tfWorldWrist3Ros;
        // Eigen::Affine3d tfWorldCameraEigen, tfWorldWrist3Eigen;
        // try {
        //     tfWorldCameraRos = tf_buffer_->lookupTransform(
        //         camera_frame_id_, world_frame_id_, rclcpp::Time(0),
        //         rclcpp::Duration::from_seconds(0.100));
        //     poseRosToEigen(tfWorldCameraRos.transform, tfWorldCameraEigen);
        //     std::cout << "TransformLister:\n translation ["
        //               << tfWorldCameraRos.transform.translation.x << ","
        //               << tfWorldCameraRos.transform.translation.y << ","
        //               << tfWorldCameraRos.transform.translation.z << "]\n"
        //               << tfWorldCameraEigen.matrix() << std::endl;

        //     tfWorldWrist3Ros = tf_buffer_->lookupTransform(
        //         wrist_frame_id_, world_frame_id_, rclcpp::Time(0),
        //         rclcpp::Duration::from_seconds(0.100));
        //     poseRosToEigen(tfWorldWrist3Ros.transform, tfWorldWrist3Eigen);
        //     std::cout << "wrist3:\n translation ["
        //               << tfWorldWrist3Ros.transform.translation.x << ","
        //               << tfWorldWrist3Ros.transform.translation.y << ","
        //               << tfWorldWrist3Ros.transform.translation.z << "]\n"
        //               << tfWorldWrist3Eigen.matrix() << std::endl;
        // } catch (tf2::TransformException& e) {
        //     std::cerr << "TransformLister exception: " << e.what() << std::endl;
        // }

        Eigen::Affine3d transform_world_camera =
            robot_->getLinkFrameTransform(camera_frame_id_);
        Eigen::Affine3d transform_world_wrist =
            robot_->getLinkFrameTransform(wrist_frame_id_);
        Eigen::Affine3d transform_world_camera_manual =
            transform_world_wrist * transform_wrist_camera_;

        RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "message frame_id \""
                << msg.header.frame_id << "\" " << "world frame \""
                << world_frame_id_ << "\"\n"
                << "transform_world_camera_moveit: camera_frame_id_ \""
                << camera_frame_id_ << "\"\n"
                << transform_world_camera.matrix() << "\n"
                << "transform_world_wrist \"" << wrist_frame_id_ << "\":\n"
                << transform_world_wrist.matrix() << "\n"
                << "transform_wrist_camera_ \"" << wrist_frame_id_ << "\":\n"
                << transform_wrist_camera_.matrix() << "\n"
                << " transform_world_wrist * transform_wrist_camera_:\n"
                << transform_world_camera_manual.matrix());

        objectsCamera_.clear();
        objectsCamera_.reserve(msg.tomato_center.size());
        objetsWorld.reserve(msg.tomato_center.size());
        for (auto &tc : msg.tomato_center)
        {
            object.center(0) = tc.x;
            object.center(1) = tc.y;
            object.center(2) = tc.z;
            object.hitNum = 1;
            object.missNum = 0;
            // Saves the object in camera coordinates for debig purpose.
            objectsCamera_.push_back(object);
            // Transform the object in world coordinate for the tracker.
            object.center = transform_world_camera * object.center;
            objetsWorld.push_back(object);
        }
        if (enableTracking_)
        {
            tracker_.update(objetsWorld);
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "tracked objects " << tracker_.getObjects().size() << ": tracking update enabled " << enableTracking_);
    }

    void TomatoPickNode::onLoopExecution()
    {
        sensor_msgs::msg::JointState jointsCurr, jointsLower6, jointsUpper6;
        bool success;
        // - Verificare che ci siano i dati sui centroidi dei pomodori
        // - Scegliere il pomodoro target (vicino, il primo del vector, ...)
        //   Eventuale calcolo distanza
        // - Calcolare la posizione del target rispetto ad un frame non dipendente
        // dal tool/camera
        //   Es. base_link, ... world_link (definito nell'ambiente di
        //   pianificazione!)
        //
        //     std::string object_id = "tomato";
        //     std::string camera_link_name = "wrist_3_link";
        //     auto trans_mat =
        //     robot_planning_.getLinkFrameTransform(camera_link_name);  // posa di
        //     "camera_link_name" rispetto al world_link (?)
        //     target_tomato_world_frame_ = trans_mat * CameraMatrix_ *
        //     target_tomato_; Nota: auto trans_mat potrebbe già essere un
        //     Eigen::Affine3d che rappresenta matrici trasf. omogenea
        //      tomatoPosWorld = worldTransfWrist * wristTransfCamera *
        //      tomatoPosCamera
        // - bozza calcolo della prima posa da raggiungere
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "current state: " << getStateName(state_));

        sensor_msgs::msg::JointState jstate;
        robot_->getJointState(jstate);
        std::cout << "joint state [";
        for (int i = 0; i < jstate.position.size(); ++i)
        {
            std::cout << jstate.position[i] << ", ";
        }
        std::cout << "]" << std::endl;

        if (state_ == State::INIT)
        {
            // Moves the robot to the HOME configuration for initial observation.
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "opening gripper: width is " << gripper_width_open_ << " mm");
            robot_->commandGripper(gripper_width_open_);
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "moving to home configuration");
            // robot_->moveToJoints(joints_home_);

            // TODO: testing moveUntilContact()
            // RCLCPP_WARN_STREAM(node_->get_logger(),
            //                    "testing moveUntilContact()");
            // Eigen::Vector3d toolSpeed, direction;
            // double acceleration;
            // toolSpeed << 0.05, 0.05, 0.05;
            // direction << 0.0, 0.10, 0.0;
            // acceleration = 0.5;
            // robot_->moveUntilContact(toolSpeed, direction, acceleration);

            state_ = State::SCENE_OBSERVATION;
            // Possibile modifica Pioldi:
            /*
            // Command the robot to move to the home joint configuration
            RCLCPP_INFO(node_->get_logger(), "State INIT: Moving to home joint
            configuration."); robot_->moveToJoints(joints_home_);

            // Update the state to START
            state_ = State::START;
            RCLCPP_INFO(node_->get_logger(), "State changed to START.");
            */
        }
        else if (state_ == State::SCENE_OBSERVATION)
        {
            // Flag enableTracking_ is set to true once the robot motion ends.
            enableTracking_ = true;

            // Checks if there are objects that are tracked.
            // TODO: improve the policy at the moment it takes objectsCandidates[0],
            // the first in the list!
            VectorObject objectsCandidates;
            tracker_.getValidObjects(objectsCandidates);
            if (!objectsCandidates.empty())
            {
                targetPosition_ = objectsCandidates[0].center;
                // Stops updating the tracker, since the robot is ready to move and
                // during motion the observation is rather noisy or not focused
                // on the scene.
                enableTracking_ = false;
                tracker_.resetCounters();
                // Computes observation configuration, moves and only then change
                // the state.
                computeLoSPose(transform_world_wrist_goal_); //
                robot_->moveToPose(transform_world_wrist_goal_);
                state_ = State::CLOSE_OBSERVATION;
            }

            // Non bisognerebbe impostare lo stato in CLOSE_OBSERVATION??

            // TODO: assegnare un ID univoco ad ogni pomodoro nel tracker, sen non è
            // èiù usato riassegarlo, numeri dinamici Pubblicare la posegoal
            // (renderla varaibile della classe?) mettere le cose nelle funzioni

            // Possibile implementazione migliorata Pioldi
            // prende il centroide più vicino al Robot (partendo dalla
            // trasformazine)
            /*
            // Checks if there are objects that are tracked.
            VectorObject objectsCandidates;
            tracker_.getValidObjects(objectsCandidates);
            if (!objectsCandidates.empty())
            {
                // Define the frame ID for the robot's current position
                std::string frame_id = "base_link";

                // Get the robot's current position
                Eigen::Affine3d robotTransform =
            robot_->getLinkFrameTransform(frame_id); Eigen::Vector3d robotPosition =
            robotTransform.translation();

                // Improve the policy: select the closest object
                auto closestObjectIt = std::min_element(objectsCandidates.begin(),
            objectsCandidates.end(),
                    [&robotPosition](const Object& a, const Object& b) {
                        return (a.center - robotPosition).norm() < (b.center -
            robotPosition).norm();
                    });

                if (closestObjectIt != objectsCandidates.end())
                {
                    targetPosition_ = closestObjectIt->center;
                    // Computes observation configuration, moves and only then
            change the state. Eigen::Affine3d poseGoal; computeLoSPose(poseGoal);
                    // tracker_.resetCounters();
                    // robot_->moveToPose(poseGoal);
                    // state_ = State::CLOSE_OBSERVATION;
                }
            }
            */
        }
        else if (state_ == State::CLOSE_OBSERVATION)
        {
            enableTracking_ = true;
            // Observes again the scene
            VectorObject objectsCandidates;
            tracker_.getValidObjects(objectsCandidates);
            if (!objectsCandidates.empty())
            {
                enableTracking_ = false;
                tracker_.resetCounters();
                targetPosition_ = objectsCandidates[0].center;
                state_ = State::APPROACH;
            }
        }
        else if (state_ == State::APPROACH)
        {
            // Moves to grasp pose at FAR distance
            computeGraspPoseAdaptable(transform_world_wrist_goal_, grasp_distance_far_, approach_dir_pitch1_);
            std::cout << "Moving toward target in [" << targetPosition_.transpose() << "] at FAR grasp pose:\n"
                      << transform_world_wrist_goal_.matrix() << std::endl;
            success = robot_->moveToPose(transform_world_wrist_goal_);

            // Reads current configuration and set last joint to -2*M_PI
            // ready for a full turn
            robot_->getJointState(jointsCurr);
            std::cout << "Curr Joints: [";
            for (int i = 0; i < 6; ++i)
            {
                std::cout << jointsCurr.position[i] << " ";
            }
            std::cout << "]" << std::endl;
            jointsLower6 = jointsCurr;
            jointsUpper6 = jointsCurr;
            jointsLower6.position[5] = -2.0 * M_PI;
            jointsUpper6.position[5] = 2.0 * M_PI;
            std::cout << "\n\n******\nLower/Upper Joint 6: " << jointsLower6.position[5] << "/" << jointsUpper6.position[5] << "\n"
                      << std::endl;
            robot_->moveToJoints(jointsLower6);

            // Moves to grasp pose at FAR distance
            computeGraspPoseAdaptable(transform_world_wrist_goal_, grasp_distance_close_, approach_dir_pitch1_);
            std::cout << "Moving toward target in [" << targetPosition_.transpose() << "] at CLOSE grasp pose:\n"
                      << transform_world_wrist_goal_.matrix() << std::endl;
            success = robot_->moveToPose(transform_world_wrist_goal_);

            // Moves to grasp pose at FAR distance
            computeGraspPoseAdaptable(transform_world_wrist_goal_, grasp_distance_close_, approach_dir_pitch2_);
            std::cout << "Moving toward target in [" << targetPosition_.transpose() << "] rotating DOWNWARD grasp pose:\n"
                      << transform_world_wrist_goal_.matrix() << std::endl;
            success = robot_->moveToPose(transform_world_wrist_goal_);

            if (success)
            {
                state_ = State::PICK;
            }
        }
        else if (state_ == State::PICK)
        {
            // Reads current configuration and set last joint to -2*M_PI
            // ready for a full turn
            robot_->getJointState(jointsCurr);
            std::cout << "Curr Joints: [";
            for (int i = 0; i < 6; ++i)
            {
                std::cout << jointsCurr.position[i] << " ";
            }
            std::cout << "]" << std::endl;
            jointsLower6 = jointsCurr;
            jointsUpper6 = jointsCurr;
            jointsLower6.position[5] = -2.0 * M_PI;
            jointsUpper6.position[5] = 2.0 * M_PI;
            std::cout << "\n\n******\nLower/Upper Joint 6: " << jointsLower6.position[5] << "/" << jointsUpper6.position[5] << "\n"
                      << std::endl;

            //robot_->commandGripper(gripper_width_closed_);

            std::cout << "POSITIVE ROTATION (gripper OPENED)\n";
            robot_->moveToJoints(jointsUpper6);

            //robot_->commandGripper(gripper_width_open_);
            std::cout << "NEGATIVE ROTATION (gripper OPENED)\n";
            robot_->moveToJoints(jointsLower6);

            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "closing gripper: width is " << gripper_width_closed_ << " mm");
            robot_->commandGripper(gripper_width_closed_);

            std::cout << "POSITIVE ROTATION (gripper CLOSED)\n";
            robot_->moveToJoints(jointsUpper6);

            std::cout << "NEGATIVE ROTATION (gripper CLOSED)\n";
            robot_->moveToJoints(jointsUpper6);robot_->moveToJoints(jointsLower6);

            // robot_->moveToJoints(jointsCurr);

            state_ = State::PLACE;
        }
        else if (state_ == State::PLACE)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "going to deposit configuration");
            robot_->moveToJoints(joints_deposit_);
        }
        else if (state_ == State::DUMMY_INIT)
        {
            RCLCPP_INFO(node_->get_logger(), "Dummy initialization");
            robot_->commandGripper(gripper_width_open_);
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "moving to home configuration");
            state_ = State::DUMMY_HOME;
            //robot_->moveToJoints(joints_home_);

        }
        else if (state_ == State::DUMMY_HOME)
        {
            RCLCPP_INFO(node_->get_logger(), "Dummy home");
            robot_->moveToJoints(joints_home_);
            //robot_->commandGripper(gripper_width_closed_);
            state_ = State::DUMMY_DEPOSIT;

        }
        else if (state_ == State::DUMMY_DEPOSIT)
        {
            RCLCPP_INFO(node_->get_logger(), "Dummy deposit");
            robot_->commandGripper(gripper_width_closed_);
            robot_->moveToJointsAsync(joints_deposit_);
            state_ = State::DUMMY_WRENCH;

        }
        else if (state_ == State::DUMMY_WRENCH)
        {
            RCLCPP_INFO(node_->get_logger(), "Dummy wrench");
            geometry_msgs::msg::Wrench wrench_value;
            robot_->getWrench(wrench_value);
            // Wrench_value is populated by robot_->getWrench()
            double force_x = wrench_value.force.x;
            double force_y = wrench_value.force.y;
            double force_z = wrench_value.force.z;

            // Calculate the norm of the force
            double norm_force = std::sqrt(force_x * force_x + force_y * force_y + force_z * force_z);

            // If the norm is greater than 25
            if (norm_force > 25)
            {
                RCLCPP_INFO(node_->get_logger(), "Force norm is greater than 25: %.2f", norm_force);
                robot_->cancel_action();
                state_ = State::DUMMY_ROTATE;
                
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Force norm is less than or equal to 25: %.2f", norm_force);
                if (robot_->is_trajectory_completed()){
                    state_ = State::DUMMY_END;
                }
                
            }
        }
        else if (state_ == State::DUMMY_ROTATE)
        {
            if(robot_->is_trajectory_completed()){

                rclcpp::sleep_for(std::chrono::milliseconds(100));
                state_ = State::DUMMY_ROTATE; 
            }
            else {

                //robot_->moveToJoints(joints_home_);
                robot_->getJointState(jointsCurr);
                std::cout << "Curr Joints: [";
                for (int i = 0; i < 6; ++i)
                {
                    std::cout << jointsCurr.position[i] << " ";
                }
                std::cout << "]" << std::endl;
                jointsLower6 = jointsCurr;
                jointsUpper6 = jointsCurr;
                jointsLower6.position[5] = -0.25 * M_PI;
                jointsUpper6.position[5] = 0.25 * M_PI;
                
                RCLCPP_INFO(node_->get_logger(), "Dummy roatate");
                std::cout << "POSITIVE ROTATION (gripper CLOSED)\n";
                robot_->moveToJoints(jointsUpper6);

                std::cout << "NEGATIVE ROTATION (gripper CLOSED)\n";
                robot_->moveToJoints(jointsUpper6);robot_->moveToJoints(jointsLower6);
                state_ = State::DUMMY_DEPOSIT;

            }
            

        }
        else if (state_ == State::DUMMY_END)
        {
            RCLCPP_INFO(node_->get_logger(), "Dummy end");

        }
        else
        {
        }

        publishTf();
    }

    void TomatoPickNode::spin()
    {
        rclcpp::spin(node_);
    }

    std::string TomatoPickNode::getStateName(const State &state)
    {
        if (state == State::INIT)
        {
            return std::string("INIT");
        }
        else if (state == State::SCENE_OBSERVATION)
        {
            return std::string("SCENE_OBSERVATION");
        }
        else if (state == State::CLOSE_OBSERVATION)
        {
            return std::string("CLOSE_OBSERVATION");
        }
        else if (state == State::APPROACH)
        {
            return std::string("APPROACH");
        }
        else if (state == State::PICK)
        {
            return std::string("PICK");
        }
        else if (state == State::PLACE)
        {
            return std::string("PLACE");
        }
        else if (state == State::DUMMY_INIT)
        {
            return std::string("DUMMY_INIT");
        }
        else if (state == State::DUMMY_HOME)
        {
            return std::string("DUMMY_HOME");
        }
        else if (state == State::DUMMY_DEPOSIT)
        {
            return std::string("DUMMY_DEPOSIT");
        }
        else if (state == State::DUMMY_WRENCH)
        {
            return std::string("DUMMY_WRENCH");
        }
        else if (state == State::DUMMY_ROTATE)
        {
            return std::string("DUMMY_ROTATE");
        }
        else if (state == State::DUMMY_END)
        {
            return std::string("DUMMY_END");
        }
        else
        {
            return std::string("INVALID");
        }
    }

    Eigen::Affine3d TomatoPickNode::readTransform(
        const std::string &position_param,
        const std::vector<double> &position_default,
        const std::string &rot_rpy_param,
        const std::vector<double> &rot_rpy_default)
    {
        Eigen::Affine3d transform;
        const double D2R = M_PI / 180.0;

        // Reads the fixed orientation of camera frame w.r.t. the wrist frame
        std::vector<double> position_value =
            node_->declare_parameter<std::vector<double>>(position_param,
                                                          position_default);
        std::vector<double> rot_rpy_value =
            node_->declare_parameter<std::vector<double>>(rot_rpy_param,
                                                          rot_rpy_default);
        if (position_value.size() == 3 && rot_rpy_value.size() == 3)
        {
            transform =
                Eigen::AngleAxisd(D2R * rot_rpy_value[2],
                                  Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(D2R * rot_rpy_value[1],
                                  Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(D2R * rot_rpy_value[0], Eigen::Vector3d::UnitX());
            transform.translate(Eigen::Vector3d(
                position_value[0], position_value[1], position_value[2]));
        }
        else
        {
            RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                "invalid size of \""
                    << position_param << "\" " << position_value.size()
                    << " or of \"" << rot_rpy_param << "\" " << rot_rpy_value.size()
                    << ": they both should be 3");
            transform = Eigen::Affine3d::Identity();
        }
        return transform;
    }

    sensor_msgs::msg::JointState TomatoPickNode::readJointState(
        const std::string &joints_param,
        const std::vector<double> &joints_default)
    {
        sensor_msgs::msg::JointState joints_value;

        joints_value.position = node_->declare_parameter<std::vector<double>>(
            joints_param, joints_default);

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

        return joints_value;
    }

    void TomatoPickNode::readVector3(
        const std::string &vec_param,
        Eigen::Vector3d &vec_value)
    {
        std::vector<double> vec_std = node_->declare_parameter<std::vector<double>>(
            vec_param, {0.0, 0.0, 0.0});
        if (vec_std.size() == 3)
        {
            vec_value << vec_std[0], vec_std[1], vec_std[2];
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "read param \"" << vec_param << "\": ["
                                           << vec_value.transpose() << "]");
    }
    
    void TomatoPickNode::selectTargetObject(Object& target, const VectorObject& objects) {
        double dist, distMin, zMax, score, scoreMax;
        int iNearest, iZMax;

        // Selects the closest to the world frame origin
        iNearest = -1;
        iZMax = -1;
        distMin = 1.0;
        zMax = 1.0;
        for (int i = 0; i < objects.size(); ++i) {
            dist = objects[i].center.norm();
            if (iNearest < 0 || dist < distMin) {
                distMin = dist;
                iNearest = i;
            }
            if (iZMax < 0 || objects[i].center(2) > zMax) {
                zMax = objects[i].center(2); 
                iZMax = i;
            }
        }

        for (int i = 0; i < objects.size(); ++i) {
            dist = objects[i].center.norm();
            score = 1.0 / (1.0 + (dist - distMin) / distMin);  
            score += 1.0 / (1.0 + (zMax - objects[i].center(2)) / zMax);
        }
    }

    void TomatoPickNode::computeLoSPose(Eigen::Affine3d &poseWristTarget)
    {
        Eigen::Affine3d transform_world_camera_goal;

        // camnbiare tutto qui come fatto sopra

        //  quando vedo un pomodoro e voglio riosservrlo devo movuermi sulla LOS
        //  (line of sight), per dire al pianificatore dove devo muovermi io devo
        // dare la posa del wrist mantenendo la camera sulla LOS

        // Gets the current poses of camera and wrist w.r.t. world frame and
        // computes their relative which is fixed and is already available
        // (but recomputed to check its value)
        Eigen::Affine3d transform_world_camera_curr =
            robot_->getLinkFrameTransform(camera_frame_id_);
        Eigen::Affine3d transform_world_wrist_curr =
            robot_->getLinkFrameTransform(wrist_frame_id_);
        // Eigen::Affine3d transform_wrist_camera_curr =
        transform_wrist_camera_curr_ =
            transform_world_wrist_curr.inverse() * transform_world_camera_curr;

        RCLCPP_ERROR_STREAM(node_->get_logger(), "comparing two estimations of transform wrist -> camera:\n"
                                                     << "  transform_wrist_camera_curr\n"
                                                     << transform_wrist_camera_curr_.matrix() << "\n"
                                                     << "  transform_wrist_camera_\n"
                                                     << transform_wrist_camera_.matrix() << "\n");

        // Computes the line-of-sight (LoS) direction from camera to the object
        // position. The direction is stored in losDir.
        Eigen::Vector3d losDir =
            targetPosition_ - transform_world_camera_curr.translation();
        losDir.normalize();
        poseTowardGoal(transform_world_camera_curr, targetPosition_, losDir,
                       observation_distance_, transform_world_camera_goal, 2);

        // Sets a new camera frame with:
        // - axis z: along losDir
        // -
        // transform_world_camera_goal.translation() =
        //     targetPosition_ - losDir * observation_distance_;
        // Eigen::Vector3d axisZ =
        //     transform_world_camera_curr.matrix().block<3, 1>(0, 2);
        // Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(axisZ,
        // losDir); transform_world_camera_goal.linear() = quat.toRotationMatrix();

        poseWristTarget =
            transform_world_camera_goal *
            transform_wrist_camera_curr_.inverse();

        RCLCPP_ERROR_STREAM(node_->get_logger(), ": transform_world_camera_goal\n"
                                                     << transform_world_camera_goal.matrix() << "\n"
                                                     << "transform_world_wrist_goal\n"
                                                     << poseWristTarget.matrix() << "\n");
        // transform_camera_wrist_curr;
    }

    void TomatoPickNode::computeGraspPoseAdaptable(Eigen::Affine3d &poseWristTarget, double distance, double approach_dir_pitch)
    {
        Eigen::Affine3d transform_world_gripper_goal;
        Eigen::Affine3d transform_world_gripper_curr, transform_world_wrist_curr, transform_gripper_wrist_curr;
        Eigen::Vector3d approach_dir_adapted;
        double approach_dir_yaw;

        // Gets the current poses of camera and wrist w.r.t. world frame and
        // computes their relative which is fixed and is already available
        // (but recomputed to check its value)
        transform_world_gripper_curr =
            robot_->getLinkFrameTransform(gripper_frame_id_);
        transform_world_wrist_curr =
            robot_->getLinkFrameTransform(wrist_frame_id_);
        transform_gripper_wrist_curr =
            transform_world_gripper_curr.inverse() * transform_world_wrist_curr;
        std::cout << "transform_gripper_wrist_curr:\n"
                  << transform_gripper_wrist_curr.matrix() << std::endl;

        approach_dir_yaw = atan2(transform_world_wrist_curr.translation()(1), transform_world_wrist_curr.translation()(0));
        approach_dir_adapted << cos(approach_dir_pitch) * cos(approach_dir_yaw),
            cos(approach_dir_pitch) * sin(approach_dir_yaw),
            sin(approach_dir_pitch);
        RCLCPP_INFO_STREAM(node_->get_logger(), ": approach direction\n"
                                                    << "  approach_dir_yaw: " << approach_dir_yaw << " (deg " << (180.0 / M_PI * approach_dir_yaw) << ")\n"
                                                    << "  approach_dir_pitch: " << approach_dir_pitch << " (deg " << (180.0 / M_PI * approach_dir_pitch) << ")\n"
                                                    << "  approach_dir: [" << approach_dir_adapted.transpose() << "]\n");

        // Computes the grasp approach direction from gripper to the object
        // position.
        // Eigen::Vector3d approachDir =
        //     targetPosition_ - transform_world_gripper_curr.translation();
        // approachDir.normalize();
        poseTowardGoal(transform_world_gripper_curr, targetPosition_, approach_dir_adapted,
                       distance, transform_world_gripper_goal, 2);

        // Sets a new camera frame with:
        // - axis z: along losDir
        // -
        // transform_world_camera_goal.translation() =
        //     targetPosition_ - losDir * observation_distance_;
        // Eigen::Vector3d axisZ =
        //     transform_world_camera_curr.matrix().block<3, 1>(0, 2);
        // Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(axisZ,
        // losDir); transform_world_camera_goal.linear() = quat.toRotationMatrix();

        poseWristTarget =
            transform_world_gripper_goal *
            transform_gripper_wrist_curr;
    }

    void TomatoPickNode::computeGraspPose(Eigen::Affine3d &poseWristTarget, double distance)
    {
        Eigen::Affine3d transform_world_gripper_goal;

        // Gets the current poses of camera and wrist w.r.t. world frame and
        // computes their relative which is fixed and is already available
        // (but recomputed to check its value)
        Eigen::Affine3d transform_world_gripper_curr =
            robot_->getLinkFrameTransform(gripper_frame_id_);
        Eigen::Affine3d transform_world_wrist_curr =
            robot_->getLinkFrameTransform(wrist_frame_id_);
        Eigen::Affine3d transform_gripper_wrist_curr =
            transform_world_gripper_curr.inverse() * transform_world_wrist_curr;
        std::cout << "transform_gripper_wrist_curr:\n"
                  << transform_gripper_wrist_curr.matrix() << std::endl;

        // Computes the grasp approach direction from gripper to the object
        // position.
        // Eigen::Vector3d approachDir =
        //     targetPosition_ - transform_world_gripper_curr.translation();
        // approachDir.normalize();
        poseTowardGoal(transform_world_gripper_curr, targetPosition_, approach_dir_,
                       distance, transform_world_gripper_goal, 2);

        // Sets a new camera frame with:
        // - axis z: along losDir
        // -
        // transform_world_camera_goal.translation() =
        //     targetPosition_ - losDir * observation_distance_;
        // Eigen::Vector3d axisZ =
        //     transform_world_camera_curr.matrix().block<3, 1>(0, 2);
        // Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(axisZ,
        // losDir); transform_world_camera_goal.linear() = quat.toRotationMatrix();

        poseWristTarget =
            transform_world_gripper_goal *
            transform_gripper_wrist_curr;
    }

    void TomatoPickNode::publishTf()
    {
        std::stringstream ss;
        int count;
        auto stampNow = node_->get_clock()->now();

        // Publishes the observed tomato in camera reference frame
        RCLCPP_INFO_STREAM(node_->get_logger(),
                           "observed " << objectsCamera_.size() << " tomatoes");
        count = 0;
        for (auto &object : objectsCamera_)
        {
            ss.str("");
            ss << "tomato_observed_" << std::setfill('0') << std::setw(2) << count;
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                ss.str() << ": position [" << object.center.transpose() << "], hit "
                         << object.hitNum << ", miss " << object.missNum);
            publishPosition(object.center, camera_frame_id_, ss.str(), stampNow);
            count++;
        }

        // Publishes the tomatoes tracked in world reference frame
        RCLCPP_INFO_STREAM(node_->get_logger(), "tracked "
                                                    << tracker_.getObjects().size()
                                                    << " tomatoes");
        count = 0;
        for (auto &object : tracker_.getObjects())
        {
            ss.str("");
            ss << "tomato_" << std::setfill('0') << std::setw(2) << count;
            RCLCPP_INFO_STREAM(
                node_->get_logger(),
                ss.str() << ": position [" << object.center.transpose() << "], hit "
                         << object.hitNum << ", miss " << object.missNum);
            publishPosition(object.center, world_frame_id_, ss.str(), stampNow);
            count++;
        }

        publishPosition(targetPosition_, world_frame_id_, std::string("target_link"), stampNow);
        publishTransform(transform_world_wrist_goal_, world_frame_id_,
                         std::string("wrist_goal_link"), stampNow);
        publishTransform(transform_world_wrist_goal_ * transform_wrist_camera_curr_, world_frame_id_,
                         std::string("camera_goal_link"), stampNow);
    }

    void TomatoPickNode::publishTransform(
        const Eigen::Affine3d &pose,
        const std::string &frame_id,
        const std::string &child_frame_id,
        const builtin_interfaces::msg::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped transform_msg;
        Eigen::Quaterniond q(pose.rotation());

        transform_msg.header.stamp = stamp; // node_->get_clock()->now();
        transform_msg.header.frame_id = frame_id;
        transform_msg.child_frame_id = child_frame_id;
        poseEigenToRos(pose, transform_msg.transform);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    void TomatoPickNode::publishPosition(
        const Eigen::Vector3d &position,
        const std::string &frame_id,
        const std::string &child_frame_id,
        const builtin_interfaces::msg::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped transform_msg;

        transform_msg.header.stamp = stamp; // node_->get_clock()->now();
        transform_msg.header.frame_id = frame_id;
        transform_msg.child_frame_id = child_frame_id;
        transform_msg.transform.translation.x = position(0);
        transform_msg.transform.translation.y = position(1);
        transform_msg.transform.translation.z = position(2);
        transform_msg.transform.rotation.x = 0.0;
        transform_msg.transform.rotation.y = 0.0;
        transform_msg.transform.rotation.z = 0.0;
        transform_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(transform_msg);
    }

} // namespace agritech_manip
