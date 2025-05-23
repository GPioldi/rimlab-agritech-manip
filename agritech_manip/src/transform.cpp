#include <agritech_manip/transform.h>

namespace agritech_manip {

void poseEigenToRos(const Eigen::Affine3d& poseEigen,
                    geometry_msgs::msg::Transform& poseMsg) {
    Eigen::Quaterniond q(poseEigen.rotation());
    poseMsg.translation.x = poseEigen.translation()(0);
    poseMsg.translation.y = poseEigen.translation()(1);
    poseMsg.translation.z = poseEigen.translation()(2);
    poseMsg.rotation.x = q.x();
    poseMsg.rotation.y = q.y();
    poseMsg.rotation.z = q.z();
    poseMsg.rotation.w = q.w();
}

void poseEigenToRos(const Eigen::Affine3d& poseEigen,
                    geometry_msgs::msg::Pose& poseMsg) {
    Eigen::Quaterniond q(poseEigen.rotation());
    poseMsg.position.x = poseEigen.translation()(0);
    poseMsg.position.y = poseEigen.translation()(1);
    poseMsg.position.z = poseEigen.translation()(2);
    poseMsg.orientation.x = q.x();
    poseMsg.orientation.y = q.y();
    poseMsg.orientation.z = q.z();
    poseMsg.orientation.w = q.w();
}

void poseRosToEigen(const geometry_msgs::msg::Transform& poseMsg,
                    Eigen::Affine3d& poseEigen) {
    Eigen::Quaterniond q(poseMsg.rotation.x, poseMsg.rotation.y,
                         poseMsg.rotation.z, poseMsg.rotation.w);
    Eigen::Vector3d t(poseMsg.translation.x, poseMsg.translation.y,
                      poseMsg.translation.z);
    poseEigen = Eigen::Affine3d::Identity();
    poseEigen.prerotate(q);
    poseEigen.pretranslate(t);
}

void poseRosToEigen(const geometry_msgs::msg::Pose& poseMsg,
                    Eigen::Affine3d& poseEigen) {
    Eigen::Quaterniond q(poseMsg.orientation.x, poseMsg.orientation.y,
                         poseMsg.orientation.z, poseMsg.orientation.w);
    Eigen::Vector3d t(poseMsg.position.x, poseMsg.position.y,
                      poseMsg.position.z);
    poseEigen = Eigen::Affine3d::Identity();
    poseEigen.prerotate(q);
    poseEigen.pretranslate(t);
}

void positionToPose(Eigen::Vector3d& position,
                    const Eigen::Affine3d& poseEigen) {}

void poseTowardGoal(const Eigen::Affine3d& poseSrc,
                    const Eigen::Vector3d& positionGoal,
                    const Eigen::Vector3d& dirGoal,
                    double distance,
                    Eigen::Affine3d& poseGoal,
                    unsigned int axisIdx) {
    Eigen::Vector3d dirGoalNorm;
    if (axisIdx < 0 || axisIdx > 2) {
        std::cerr << __FILE__ << "," << __LINE__ << ": invalid axis " << axisIdx
                  << ": it must be 0 (axis X), 1 (axis Y) or 2 (axis Z)"
                  << std::endl;
        exit(-1);
    }
    if (dirGoal.norm() < 1e-6) {
        std::cerr << __FILE__ << "," << __LINE__ << ": invalid dirGoal with norm close to ZERO! norm = " << dirGoal.norm() << std::endl;
        return; 
    }
    dirGoalNorm = dirGoal / dirGoal.norm();
    Eigen::Vector3d zSrc = poseSrc.linear().block<3, 1>(0, axisIdx);
    std::cout << "dirGoalNorm [" << dirGoalNorm.transpose() << "]  zSrc [" << zSrc.transpose() << "]" << std::endl;
    poseGoal.translation() = positionGoal - distance * dirGoalNorm;
    poseGoal.linear() =
        Eigen::Quaterniond::FromTwoVectors(zSrc, dirGoal) * poseSrc.rotation();
}

}  // end of namespace agritech_manip