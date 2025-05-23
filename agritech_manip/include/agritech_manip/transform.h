#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <Eigen/Dense>
#include <iostream>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace agritech_manip {

/**
 * @brief Converts a pose in Eigen::Affine3d format to ROS
 * geometry_msgs::msg::Transform.
 *
 * @param poseEigen pose in Eigen format
 * @param poseMsg pose in ROS message Transform format
 */
void poseEigenToRos(const Eigen::Affine3d& poseEigen,
                    geometry_msgs::msg::Transform& poseMsg);

/**
 * @brief Converts a pose in Eigen::Affine3d format to ROS
 * geometry_msgs::msg::Pose.
 *
 * @param poseEigen pose in Eigen format
 * @param poseMsg pose in ROS message Pose format
 */
void poseEigenToRos(const Eigen::Affine3d& poseEigen,
                    geometry_msgs::msg::Pose& poseMsg);

/**
 * @brief Converts a pose in ROS geometry_msgs::msg::Transform to
 * Eigen::Affine3d format.
 *
 * @param poseMsg pose in ROS message Transform format
 * @param poseEigen pose in Eigen format
 */
void poseRosToEigen(const geometry_msgs::msg::Transform& poseMsg,
                    Eigen::Affine3d& poseEigen);

/**
 * @brief Converts a pose in ROS geometry_msgs::msg::Pose to
 * Eigen::Affine3d format.
 *
 * @param poseMsg pose in ROS message Pose format
 * @param poseEigen pose in Eigen format
 */
void poseRosToEigen(const geometry_msgs::msg::Pose& poseMsg,
                    Eigen::Affine3d& poseEigen);

/**
 * @brief Computes the poseGoal satisfying the following conditions:
 * 1) The poseGoal origin is at given distance from a target in positionGoal.
 * 2) The axis axisIdx (by default axisIdx = 2 corresponding to axis Z) of
 *    poseGoal is oriented along direction dirGoal.
 * 3) The condition 2) is satisfied by multiple orientations of poseGoal.
 *    Thus, the orientation of poseGoal is selected such that the rotation
 *    between the initial pose poseSrc and poseGoal is minimal.
 *    Namely, the relative rotation from poseSrc to poseGoal is the one
 *    aligning axis axisIdx of poseSrc to dirGoal.
 *
 * @param poseSrc original source orientation
 * @param positionGoal position of the target toward which the new frame is
 * directed
 * @param dirGoal desired orientation of axis axisIdx
 * @param distance distance from target of poseGoal
 * @param poseGoal the pose of the goal
 * @param axisIdx the index of the axis
 */
void poseTowardGoal(const Eigen::Affine3d& poseSrc,
                    const Eigen::Vector3d& positionGoal,
                    const Eigen::Vector3d& dirGoal,
                    double distance,
                    Eigen::Affine3d& poseGoal,
                    unsigned int axisIdx = 2);

}  // namespace agritech_manip

#endif