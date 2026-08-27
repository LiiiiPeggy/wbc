// ################################
// C++: EE kinematics API compile check begin
// ################################
#include <mm_config/ee_kinematics_utils.hpp>
#include <mm_config/mm_config.hpp>

int main() {
    remani_planner::MMConfig config;
    Eigen::Vector3d car_state = Eigen::Vector3d::Zero();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Matrix<double, 6, 9> J;
    config.getEeJacobian(car_state, q, J);

    Eigen::Matrix<double, 6, 1> e;
    remani_planner::poseError(Eigen::Matrix4d::Identity(),
                              Eigen::Matrix4d::Identity(), e);
    return remani_planner::allFinite(e) && remani_planner::allFinite(J) ? 0 : 1;
}
// ################################
// C++: EE kinematics API compile check end
// ################################
