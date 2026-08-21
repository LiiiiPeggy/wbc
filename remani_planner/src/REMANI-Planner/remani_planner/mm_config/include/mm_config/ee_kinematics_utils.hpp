// ################################
// C++: Eigen-only EE kinematics utilities begin
// ################################
#ifndef _EE_KINEMATICS_UTILS_HPP_
#define _EE_KINEMATICS_UTILS_HPP_

#include <Eigen/Core>
#include <cmath>

namespace remani_planner
{

inline double wrapToPi(double a){
    const double pi = 3.14159265358979323846;
    const double two_pi = 2.0 * pi;
    a = std::fmod(a + pi, two_pi);
    if(a < 0.0){
        a += two_pi;
    }
    return a - pi;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v){
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return S;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d &S){
    return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

inline Eigen::Vector3d rotationLog(const Eigen::Matrix3d &R){
    if(!R.allFinite()){
        return Eigen::Vector3d::Zero();
    }

    const double pi = 3.14159265358979323846;
    double cos_theta = 0.5 * (R.trace() - 1.0);
    cos_theta = std::fmax(-1.0, std::fmin(1.0, cos_theta));
    const double theta = std::acos(cos_theta);

    if(theta < 1e-8){
        return 0.5 * vee(R - R.transpose());
    }

    if(pi - theta < 1e-6){
        Eigen::Vector3d axis;
        axis.x() = std::sqrt(std::fmax(0.0, 0.5 * (R(0, 0) + 1.0)));
        axis.y() = std::sqrt(std::fmax(0.0, 0.5 * (R(1, 1) + 1.0)));
        axis.z() = std::sqrt(std::fmax(0.0, 0.5 * (R(2, 2) + 1.0)));

        if(axis.x() >= axis.y() && axis.x() >= axis.z() && axis.x() > 1e-8){
            axis.y() = (R(0, 1) + R(1, 0)) / (4.0 * axis.x());
            axis.z() = (R(0, 2) + R(2, 0)) / (4.0 * axis.x());
        }else if(axis.y() >= axis.z() && axis.y() > 1e-8){
            axis.x() = (R(0, 1) + R(1, 0)) / (4.0 * axis.y());
            axis.z() = (R(1, 2) + R(2, 1)) / (4.0 * axis.y());
        }else if(axis.z() > 1e-8){
            axis.x() = (R(0, 2) + R(2, 0)) / (4.0 * axis.z());
            axis.y() = (R(1, 2) + R(2, 1)) / (4.0 * axis.z());
        }else{
            return Eigen::Vector3d::Zero();
        }

        const double norm = axis.norm();
        if(!std::isfinite(norm) || norm < 1e-12){
            return Eigen::Vector3d::Zero();
        }
        axis /= norm;
        return theta * axis;
    }

    const double sin_theta = std::sin(theta);
    Eigen::Vector3d result = (theta / (2.0 * sin_theta)) * vee(R - R.transpose());
    return result.allFinite() ? result : Eigen::Vector3d::Zero();
}

inline void poseError(const Eigen::Matrix4d &T_now,
                      const Eigen::Matrix4d &T_des,
                      Eigen::Matrix<double, 6, 1> &e){
    e.template head<3>() = T_des.template block<3, 1>(0, 3)
                         - T_now.template block<3, 1>(0, 3);
    const Eigen::Matrix3d R_error =
        T_des.template block<3, 3>(0, 0)
        * T_now.template block<3, 3>(0, 0).transpose();
    e.template tail<3>() = rotationLog(R_error);
}

inline bool allFinite(const Eigen::Matrix<double, 6, 1> &e){
    return e.allFinite();
}

inline bool allFinite(const Eigen::Matrix<double, 6, 9> &J){
    return J.allFinite();
}

} // namespace remani_planner

#endif
// ################################
// C++: Eigen-only EE kinematics utilities end
// ################################
