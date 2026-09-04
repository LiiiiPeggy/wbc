#include "fake_moma/moma_param.h"

#include <sstream>

#include <xmlrpcpp/XmlRpcValue.h>

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

// ################################
// C++: Parse AG95 fixed-closed collision envelope from global /moma/*
// ################################
void loadAg95Spheres(const ros::NodeHandle& root_nh, std::vector<CollisionSphere>& spheres)
{
    XmlRpc::XmlRpcValue ag95_spheres;
    if (!root_nh.getParam("moma/ag95_spheres", ag95_spheres))
    {
        return;
    }
    if (ag95_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray || ag95_spheres.size() == 0)
    {
        throw std::runtime_error("Global parameter /moma/ag95_spheres must be a non-empty array");
    }

    spheres.clear();
    for (int idx = 0; idx < ag95_spheres.size(); ++idx)
    {
        if (ag95_spheres[idx].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            throw std::runtime_error("Global parameter /moma/ag95_spheres entries must be maps");
        }

        CollisionSphere sphere;
        const auto& entry = ag95_spheres[idx];
        if (!entry.hasMember("xyz") || !entry.hasMember("obstacle_radius")
            || !entry.hasMember("self_radius"))
        {
            throw std::runtime_error(
                "Global parameter /moma/ag95_spheres entries require xyz, obstacle_radius, self_radius");
        }

        if (entry["xyz"].getType() != XmlRpc::XmlRpcValue::TypeArray
            || entry["xyz"].size() != 3)
        {
            throw std::runtime_error("Global parameter /moma/ag95_spheres xyz must contain 3 values");
        }

        sphere.local_offset << static_cast<double>(entry["xyz"][0]),
            static_cast<double>(entry["xyz"][1]),
            static_cast<double>(entry["xyz"][2]);
        sphere.obstacle_radius = static_cast<double>(entry["obstacle_radius"]);
        sphere.self_radius = static_cast<double>(entry["self_radius"]);
        sphere.link_id = 0;
        spheres.push_back(sphere);
    }
}

// ################################
// C++: Parse upper-box obstacle multi-sphere layout from global /moma/box_obstacle
// ################################
void loadBoxObstacle(const ros::NodeHandle& root_nh, MomaParam& profile)
{
    if (!root_nh.getParam("moma/box_obstacle/enabled", profile.box_obstacle_enabled_))
    {
        profile.box_obstacle_enabled_ = false;
        return;
    }
    if (!profile.box_obstacle_enabled_)
    {
        return;
    }

    root_nh.getParam("moma/box_obstacle/margin", profile.box_obstacle_margin_);

    XmlRpc::XmlRpcValue spheres;
    if (root_nh.getParam("moma/box_obstacle/spheres", spheres))
    {
        if (spheres.getType() != XmlRpc::XmlRpcValue::TypeArray || spheres.size() == 0)
        {
            throw std::runtime_error("Global parameter /moma/box_obstacle/spheres must be a non-empty array");
        }

        profile.base_obstacle_proxies_.clear();
        for (int idx = 0; idx < spheres.size(); ++idx)
        {
            if (spheres[idx].getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                throw std::runtime_error("Global parameter /moma/box_obstacle/spheres entries must be maps");
            }

            CollisionSphere sphere;
            const auto& entry = spheres[idx];
            if (!entry.hasMember("local_offset") || !entry.hasMember("obstacle_radius"))
            {
                throw std::runtime_error(
                    "Global parameter /moma/box_obstacle/spheres entries require "
                    "local_offset and obstacle_radius");
            }

            if (entry["local_offset"].getType() != XmlRpc::XmlRpcValue::TypeArray
                || entry["local_offset"].size() != 3)
            {
                throw std::runtime_error(
                    "Global parameter /moma/box_obstacle/spheres local_offset must contain 3 values");
            }

            sphere.local_offset << static_cast<double>(entry["local_offset"][0]),
                static_cast<double>(entry["local_offset"][1]),
                static_cast<double>(entry["local_offset"][2]);
            sphere.obstacle_radius = static_cast<double>(entry["obstacle_radius"]);
            sphere.self_radius = 0.0;
            sphere.link_id = -1;
            profile.base_obstacle_proxies_.push_back(sphere);
        }
        return;
    }

    std::vector<double> grid_x;
    std::vector<double> grid_y;
    std::vector<double> grid_z;
    double obstacle_radius = 0.0;
    if (!root_nh.getParam("moma/box_obstacle/grid_x", grid_x)
        || !root_nh.getParam("moma/box_obstacle/grid_y", grid_y)
        || !root_nh.getParam("moma/box_obstacle/grid_z", grid_z)
        || !root_nh.getParam("moma/box_obstacle/obstacle_radius", obstacle_radius))
    {
        throw std::runtime_error(
            "Global /moma/box_obstacle requires spheres[] or grid_x/grid_y/grid_z + obstacle_radius");
    }
    if (obstacle_radius <= 0.0)
    {
        throw std::runtime_error("Global /moma/box_obstacle/obstacle_radius must be positive");
    }

    profile.base_obstacle_proxies_.clear();
    for (double x : grid_x)
    {
        for (double y : grid_y)
        {
            for (double z : grid_z)
            {
                CollisionSphere sphere;
                sphere.local_offset << x, y, z;
                sphere.obstacle_radius = obstacle_radius;
                sphere.self_radius = 0.0;
                sphere.link_id = -1;
                profile.base_obstacle_proxies_.push_back(sphere);
            }
        }
    }
}

// ################################
// C++: Parse profile mesh parts and their link-relative visual transforms
// ################################
double xmlNumber(const XmlRpc::XmlRpcValue& value, const std::string& name)
{
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        return static_cast<int>(value);
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        return static_cast<double>(value);
    }
    throw std::runtime_error(name + " must contain numeric values");
}

Eigen::Vector3d xmlVector3(const XmlRpc::XmlRpcValue& value, const std::string& name)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
    {
        throw std::runtime_error(name + " must contain exactly three numeric values");
    }
    return Eigen::Vector3d(xmlNumber(value[0], name), xmlNumber(value[1], name),
                           xmlNumber(value[2], name));
}

Eigen::Matrix3d rpyToRotation(const Eigen::Vector3d& rpy)
{
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

MeshRole parseMeshRole(const XmlRpc::XmlRpcValue& value, const std::string& name)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        throw std::runtime_error(name + " must be a string");
    }
    const std::string role = static_cast<std::string>(value);
    if (role == "base")
        return MeshRole::Base;
    if (role == "arm_base")
        return MeshRole::ArmBase;
    if (role == "arm_link")
        return MeshRole::ArmLink;
    if (role == "ag95")
        return MeshRole::Ag95;
    throw std::runtime_error(name + " has unsupported role: " + role);
}

void loadMeshParts(const ros::NodeHandle& root_nh, std::vector<MeshPart>& mesh_parts)
{
    XmlRpc::XmlRpcValue entries;
    getRequired(root_nh, "mesh_parts", entries);
    if (entries.getType() != XmlRpc::XmlRpcValue::TypeArray || entries.size() == 0)
    {
        throw std::runtime_error("Global parameter /moma/mesh_parts must be a non-empty array");
    }

    mesh_parts.clear();
    mesh_parts.reserve(entries.size());
    for (int entry_idx = 0; entry_idx < entries.size(); ++entry_idx)
    {
        const XmlRpc::XmlRpcValue& entry = entries[entry_idx];
        const std::string name = "Global parameter /moma/mesh_parts[" + std::to_string(entry_idx) + "]";
        if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || !entry.hasMember("role")
            || !entry.hasMember("file") || !entry.hasMember("xyz") || !entry.hasMember("rpy"))
        {
            throw std::runtime_error(name + " requires role, file, xyz, and rpy");
        }
        if (entry["file"].getType() != XmlRpc::XmlRpcValue::TypeString
            || static_cast<std::string>(entry["file"]).empty())
        {
            throw std::runtime_error(name + ".file must be a non-empty string");
        }

        MeshPart part;
        part.role = parseMeshRole(entry["role"], name + ".role");
        part.file = static_cast<std::string>(entry["file"]);
        if (part.role == MeshRole::ArmLink)
        {
            if (!entry.hasMember("index"))
            {
                throw std::runtime_error(name + " arm_link requires index");
            }
            const double index = xmlNumber(entry["index"], name + ".index");
            if (index < 0.0 || std::floor(index) != index)
            {
                throw std::runtime_error(name + ".index must be a non-negative integer");
            }
            part.index = static_cast<size_t>(index);
        }

        part.link_T_visual.block<3, 3>(0, 0) = rpyToRotation(xmlVector3(entry["rpy"], name + ".rpy"));
        part.link_T_visual.block<3, 1>(0, 3) = xmlVector3(entry["xyz"], name + ".xyz");
        // ################################
        // C++: Optional millimetre-to-metre mesh scale, default 1
        // ################################
        if (entry.hasMember("scale"))
        {
            part.scale = xmlVector3(entry["scale"], name + ".scale");
            if (part.scale.minCoeff() <= 0.0)
            {
                throw std::runtime_error(name + ".scale must be strictly positive");
            }
        }
        mesh_parts.push_back(part);
    }
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
    loadMeshParts(root_nh, profile.mesh_parts);

    // ################################
    // C++: Optional CAD visual root (defaults to identity)
    // ################################
    {
        std::vector<double> visual_xyz = {0.0, 0.0, 0.0};
        std::vector<double> visual_rpy = {0.0, 0.0, 0.0};
        root_nh.getParam("moma/visual/base_xyz", visual_xyz);
        root_nh.getParam("moma/visual/base_rpy", visual_rpy);
        if (visual_xyz.size() != 3 || visual_rpy.size() != 3)
        {
            throw std::runtime_error("Global /moma/visual/base_xyz and base_rpy must contain 3 values");
        }
        profile.visual_base_xyz = Eigen::Map<const Eigen::Vector3d>(visual_xyz.data());
        profile.visual_base_rpy = Eigen::Map<const Eigen::Vector3d>(visual_rpy.data());
    }

    if (profile.kinematics == KinematicsType::Cr10)
    {
        getRequired(root_nh, "obstacle_thickness", profile.obstacle_thickness);
        getRequired(root_nh, "self_thickness", profile.self_thickness);
        getRequired(root_nh, "sphere_spacing_factor", profile.sphere_spacing_factor);
        if (root_nh.getParam("moma/chassis_ignore_link_ids", values))
        {
            profile.chassis_ignore_link_ids.clear();
            profile.chassis_ignore_link_ids.reserve(values.size());
            for (double value : values)
            {
                profile.chassis_ignore_link_ids.push_back(static_cast<int>(value));
            }
        }
        loadAg95Spheres(root_nh, profile.ag95_spheres);
        loadBoxObstacle(root_nh, profile);
    }
    else
    {
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
        buildCollisionProxies();
        buildCollisionIgnoreMatrix();
        // ################################
        // C++: Upper-box obstacle proxies stay outside collision_matrix
        // ################################
        buildBaseObstacleProxies();
        return;
    }

    default_colli_point_radius = colli_point_radius;
    std::vector<Eigen::Vector4d> cpts = getColliPts(Eigen::VectorXd::Zero(3 + dof_num));
    colli_self_radii_.clear();
    colli_self_radii_.reserve(cpts.size());
    for (const Eigen::Vector4d& colli_pt : cpts)
    {
        colli_self_radii_.push_back(colli_pt[3]);
    }
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
// C++: Finalize profile visualization attachments after parsing
// ################################
void MomaParam::finalizeVisualization()
{
    // ################################
    // C++: Build T_base_visual from YAML visual/base_xyz + base_rpy
    // ################################
    visual_root_T_.setIdentity();
    visual_root_T_.block<3, 3>(0, 0) =
        (Eigen::AngleAxisd(visual_base_rpy.z(), Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(visual_base_rpy.y(), Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(visual_base_rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    visual_root_T_.block<3, 1>(0, 3) = visual_base_xyz;
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
        if (obstacle_thickness <= 0.0 || self_thickness <= 0.0 || sphere_spacing_factor <= 0.0)
        {
            throw std::runtime_error("Global /moma CR10 collision thickness values must be positive");
        }
        if (ag95_spheres.empty())
        {
            throw std::runtime_error("Global /moma/ag95_spheres must contain at least one sphere");
        }
        if (link_length.size() != static_cast<Eigen::Index>(dof_num))
        {
            throw std::runtime_error("Global /moma arm/link_length must match dof_num for CR10 collision");
        }
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
// C++: Validate every mesh role against the active typed kinematics profile
// ################################
void MomaParam::validateVisualization() const
{
    bool has_base = false;
    bool has_ag95 = false;
    for (const MeshPart& part : mesh_parts)
    {
        if (part.file.empty())
        {
            throw std::runtime_error("Global /moma mesh part file must not be empty");
        }
        if (part.role == MeshRole::Base)
        {
            has_base = true;
        }
        if (part.role == MeshRole::ArmLink && part.index >= dof_num)
        {
            throw std::runtime_error("Global /moma arm_link mesh index exceeds dof_num");
        }
        if (part.role == MeshRole::Ag95)
        {
            has_ag95 = true;
        }
    }
    if (!has_base)
    {
        throw std::runtime_error("Global /moma/mesh_parts requires a base mesh");
    }
    if (kinematics == KinematicsType::Cr10 && !has_ag95)
    {
        throw std::runtime_error("Global /moma/mesh_parts requires an AG95 mesh for CR10");
    }
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
