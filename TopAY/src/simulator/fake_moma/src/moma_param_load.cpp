#include "fake_moma/moma_param.h"

#include <sstream>

namespace
{
// ################################
// C++: Required global /moma parameter readers
// ################################
template <typename T>
void getRequired(const ros::NodeHandle& root_nh, const std::string& name, T& value)
{
    if (!root_nh.getParam("moma/" + name, value))
    {
        throw std::runtime_error("Missing required global parameter: /moma/" + name);
    }
}

// ################################
// C++: Convert ROS arrays into Eigen vectors
// ################################
Eigen::VectorXd toVector(const std::vector<double>& values, const std::string& name)
{
    if (values.empty())
    {
        throw std::runtime_error("Global parameter /moma/" + name + " must not be empty");
    }

    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

// ################################
// C++: Convert row-major ROS arrays into Eigen matrices
// ################################
Eigen::MatrixX3d toMatrixX3(const std::vector<double>& values, size_t rows,
                            const std::string& name)
{
    if (values.size() != rows * 3)
    {
        throw std::runtime_error("Global parameter /moma/" + name
                                 + " must contain dof_num * 3 values");
    }

    Eigen::MatrixX3d result(rows, 3);
    for (size_t row = 0; row < rows; ++row)
    {
        for (size_t col = 0; col < 3; ++col)
        {
            result(row, col) = values[row * 3 + col];
        }
    }
    return result;
}
}  // namespace

// ################################
// C++: MomaParam::fromRos reads global /moma/*
// ################################
MomaParam MomaParam::fromRos(const ros::NodeHandle& root_nh)
{
    MomaParam profile;
    std::string kinematics_name;
    int dof_num = 0;
    std::vector<double> values;

    getRequired(root_nh, "robot_name", profile.robot_name);
    getRequired(root_nh, "kinematics", kinematics_name);
    getRequired(root_nh, "dof_num", dof_num);
    if (dof_num <= 0)
    {
        throw std::runtime_error("Global parameter /moma/dof_num must be positive");
    }
    profile.dof_num = static_cast<size_t>(dof_num);

    if (kinematics_name == "topay_alt")
    {
        profile.kinematics = KinematicsType::TopayAlt;
    }
    else if (kinematics_name == "cr10")
    {
        profile.kinematics = KinematicsType::Cr10;
    }
    else
    {
        throw std::runtime_error("Unsupported global parameter /moma/kinematics: "
                                 + kinematics_name);
    }

    getRequired(root_nh, "chassis/length", profile.chassis_length);
    getRequired(root_nh, "chassis/width", profile.chassis_width);
    getRequired(root_nh, "chassis/height", profile.chassis_height);
    getRequired(root_nh, "chassis/collision_radius", profile.chassis_colli_radius);
    getRequired(root_nh, "limits/max_v", profile.max_v);
    getRequired(root_nh, "limits/max_a", profile.max_a);
    getRequired(root_nh, "limits/max_w", profile.max_w);
    getRequired(root_nh, "limits/max_dw", profile.max_dw);
    getRequired(root_nh, "arm/cylinder_radius", profile.cylinder_radius);

    getRequired(root_nh, "mount/relative_R", values);
    if (values.size() != 9)
    {
        throw std::runtime_error("Global parameter /moma/mount/relative_R must contain 9 values");
    }
    profile.relative_R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        values.data());

    getRequired(root_nh, "mount/relative_t", values);
    if (values.size() != 3)
    {
        throw std::runtime_error("Global parameter /moma/mount/relative_t must contain 3 values");
    }
    profile.relative_t = Eigen::Map<const Eigen::Vector3d>(values.data());

    getRequired(root_nh, "arm/link_length", values);
    profile.link_length = toVector(values, "arm/link_length");
    getRequired(root_nh, "arm/joint_pos_limit_min", values);
    profile.joint_pos_limit_min = toVector(values, "arm/joint_pos_limit_min");
    getRequired(root_nh, "arm/joint_pos_limit_max", values);
    profile.joint_pos_limit_max = toVector(values, "arm/joint_pos_limit_max");
    getRequired(root_nh, "arm/joint_vel_limit", values);
    profile.joint_vel_limit = toVector(values, "arm/joint_vel_limit");
    getRequired(root_nh, "arm/joint_acc_limit", values);
    profile.joint_acc_limit = toVector(values, "arm/joint_acc_limit");
    getRequired(root_nh, "arm/joint_eff_limit", values);
    profile.joint_eff_limit = toVector(values, "arm/joint_eff_limit");
    getRequired(root_nh, "arm/joint_offset", values);
    profile.joint_offset = toMatrixX3(values, profile.dof_num, "arm/joint_offset");
    getRequired(root_nh, "arm/joint_dof_axis", values);
    profile.joint_dof_axis = toMatrixX3(values, profile.dof_num, "arm/joint_dof_axis");

    getRequired(root_nh, "collision/link_length", values);
    profile.colli_length = toVector(values, "collision/link_length");
    getRequired(root_nh, "collision/points", values);
    profile.colli_points = toVector(values, "collision/points");
    getRequired(root_nh, "collision/point_radius", values);
    profile.colli_point_radius = toVector(values, "collision/point_radius");

    // ################################
    // C++: Preserve legacy minimum tracer collision sphere radius
    // ################################
    for (int i = 0; i < profile.colli_point_radius.size(); ++i)
    {
        if (profile.colli_point_radius(i) > 1e-4
            && profile.colli_point_radius(i) < profile.cylinder_radius)
        {
            profile.colli_point_radius(i) = profile.cylinder_radius;
        }
    }

    // ################################
    // C++: Validate loaded dimensions before collision finalization indexes arrays
    // ################################
    profile.validateCore();
    profile.validateKinematics();
    profile.validateCollision();
    profile.finalizeKinematics();
    profile.finalizeCollision();
    profile.finalizeVisualization();
    profile.validateVisualization();
    return profile;
}

// ################################
// C++: Convert kinematics enum to profile name
// ################################
const char* MomaParam::kinematicsName(KinematicsType type)
{
    switch (type)
    {
        case KinematicsType::TopayAlt:
            return "topay_alt";
        case KinematicsType::Cr10:
            return "cr10";
    }
    return "unknown";
}

// ################################
// C++: Finalize kinematics backend after profile validation
// ################################
void MomaParam::finalizeKinematics()
{
    if (kinematics == KinematicsType::Cr10)
    {
        initCr10FixedTransforms();
    }
}

// ################################
// C++: Rebuild collision-derived data after profile loading
// ################################
void MomaParam::finalizeCollision()
{
    if (kinematics == KinematicsType::Cr10)
    {
        return;
    }

    default_colli_point_radius = colli_point_radius;
    std::vector<Eigen::Vector4d> cpts = getColliPts(Eigen::VectorXd::Zero(3 + dof_num));
    colli_link_map.resize(cpts.size());
    colli_link_map << 0, 0, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7;
    collision_matrix.resize(cpts.size(), cpts.size());
    collision_matrix.setConstant(-1);
    for (size_t i = 0; i < cpts.size(); ++i)
    {
        for (size_t j = i; j < cpts.size(); ++j)
        {
            if (i == j)
            {
                collision_matrix(i, j) = 1;
            }
            const double distance = (cpts[i].head(3) - cpts[j].head(3)).norm();
            if (distance < cpts[i][3] + cpts[j][3])
            {
                collision_matrix(i, j) = collision_matrix(j, i) = 1;
            }
        }
    }
}

// ################################
// C++: Reserve visualization finalization for later profiles
// ################################
void MomaParam::finalizeVisualization()
{
}

// ################################
// C++: Validate profile fields shared by all robots
// ################################
void MomaParam::validateCore() const
{
    if (robot_name.empty() || dof_num == 0)
    {
        throw std::runtime_error("Global /moma profile needs robot_name and positive dof_num");
    }
    if (chassis_length <= 0.0 || chassis_width <= 0.0 || chassis_height <= 0.0
        || chassis_colli_radius <= 0.0 || max_v <= 0.0 || max_a <= 0.0
        || max_w <= 0.0 || max_dw <= 0.0 || cylinder_radius <= 0.0)
    {
        throw std::runtime_error("Global /moma profile has non-positive chassis, limit, or radius");
    }
}

// ################################
// C++: Validate kinematics arrays for the loaded backend
// ################################
void MomaParam::validateKinematics() const
{
    if (kinematics == KinematicsType::TopayAlt && dof_num != 7)
    {
        throw std::runtime_error("topay_alt profile requires dof_num == 7");
    }
    if (kinematics == KinematicsType::Cr10 && dof_num != 6)
    {
        throw std::runtime_error("cr10 profile requires dof_num == 6");
    }
    if (link_length.size() != static_cast<Eigen::Index>(dof_num)
        || joint_pos_limit_min.size() != static_cast<Eigen::Index>(dof_num)
        || joint_pos_limit_max.size() != static_cast<Eigen::Index>(dof_num)
        || joint_vel_limit.size() != static_cast<Eigen::Index>(dof_num)
        || joint_acc_limit.size() != static_cast<Eigen::Index>(dof_num)
        || joint_eff_limit.size() != static_cast<Eigen::Index>(dof_num)
        || joint_offset.rows() != static_cast<Eigen::Index>(dof_num)
        || joint_dof_axis.rows() != static_cast<Eigen::Index>(dof_num))
    {
        throw std::runtime_error("Global /moma arm arrays do not match dof_num");
    }
}

// ################################
// C++: Validate collision arrays required by existing consumers
// ################################
void MomaParam::validateCollision() const
{
    if (kinematics == KinematicsType::Cr10)
    {
        return;
    }

    const Eigen::Index expected = static_cast<Eigen::Index>(2 * (dof_num + 1));
    if (colli_length.size() != static_cast<Eigen::Index>(dof_num + 1)
        || colli_points.size() != expected || colli_point_radius.size() != expected)
    {
        throw std::runtime_error("Global /moma collision arrays do not match dof_num");
    }
}

// ################################
// C++: Validate visualization data when visualization profiles arrive
// ################################
void MomaParam::validateVisualization() const
{
}

// ################################
// C++: Validate complete global /moma robot profile
// ################################
void MomaParam::validateAll() const
{
    validateCore();
    validateKinematics();
    validateCollision();
    validateVisualization();
}
