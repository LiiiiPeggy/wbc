#include "fake_moma/moma_param.h"

#include <algorithm>
#include <cmath>

namespace
{
// ################################
// C++: URDF-style fixed-axis rotation helpers for CR10
// ################################
Eigen::Matrix3d urdfRpy(double roll, double pitch, double yaw)
{
    const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q.toRotationMatrix();
}

Eigen::Matrix4d makeFixedTransform(double x, double y, double z,
                                   double roll, double pitch, double yaw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = urdfRpy(roll, pitch, yaw);
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return transform;
}

Eigen::Matrix4d baseTransform2d(double x, double y, double yaw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 0) = std::cos(yaw);
    transform(0, 1) = -std::sin(yaw);
    transform(1, 0) = std::sin(yaw);
    transform(1, 1) = std::cos(yaw);
    transform(0, 3) = x;
    transform(1, 3) = y;
    return transform;
}

Eigen::Matrix4d mountTransform(const Eigen::Matrix3d& relative_R,
                               const Eigen::Vector3d& relative_t)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = relative_R;
    transform.block<3, 1>(0, 3) = relative_t;
    return transform;
}

void cr10JointTransform(const Eigen::Matrix4d& fixed, double theta,
                        Eigen::Matrix4d& transform, Eigen::Matrix4d& transform_grad)
{
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    Eigen::Matrix4d rz = Eigen::Matrix4d::Identity();
    rz(0, 0) = cos_theta;
    rz(0, 1) = -sin_theta;
    rz(1, 0) = sin_theta;
    rz(1, 1) = cos_theta;

    Eigen::Matrix4d drz = Eigen::Matrix4d::Zero();
    drz(0, 0) = -sin_theta;
    drz(0, 1) = -cos_theta;
    drz(1, 0) = cos_theta;
    drz(1, 1) = -sin_theta;

    transform = fixed * rz;
    transform_grad = fixed * drz;
}

double eePoseDirectionalGrad(const Eigen::Matrix4d& transform_grad,
                             const Eigen::VectorXd& ee_grad)
{
    const Eigen::Matrix3d rotation_grad = transform_grad.block<3, 3>(0, 0);
    return ee_grad.head(3).dot(transform_grad.block<3, 1>(0, 3))
           + ee_grad.segment(3, 3).dot(rotation_grad.row(0))
           + ee_grad.tail(3).dot(rotation_grad.row(1));
}
}  // namespace

// ################################
// C++: Populate CR10 fixed joint origins from URDF
// ################################
void MomaParam::initCr10FixedTransforms()
{
    const double xyz_rpy[6][6] = {
        {0.0, 0.0, 0.1765, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.5708, 1.5708, 0.0},
        {-0.607, 0.0, 0.0, 0.0, 0.0, 0.0},
        {-0.568, 0.0, 0.191, 0.0, 0.0, -1.5708},
        {0.0, -0.125, 0.0, 1.5708, 0.0, 0.0},
        {0.0, 0.1084, 0.0, -1.5708, 0.0, 0.0},
    };

    for (size_t joint = 0; joint < cr10_joint_fixed_.size(); ++joint)
    {
        cr10_joint_fixed_[joint] = makeFixedTransform(
            xyz_rpy[joint][0], xyz_rpy[joint][1], xyz_rpy[joint][2],
            xyz_rpy[joint][3], xyz_rpy[joint][4], xyz_rpy[joint][5]);
    }
}

// ################################
// C++: Dispatch typed link transforms by kinematics backend
// ################################
KinematicResult MomaParam::getLinkTransforms(const Eigen::VectorXd& state) const
{
    if (kinematics == KinematicsType::Cr10)
    {
        return getLinkTransformsCr10(state);
    }
    return getLinkTransformsTopayAlt(state);
}

// ################################
// C++: CR10 exact transform chain in base frame (no chassis_height stacking)
// ################################
KinematicResult MomaParam::getLinkTransformsCr10(const Eigen::VectorXd& state) const
{
    KinematicResult result;
    result.arm_link_T.resize(dof_num);

    const Eigen::Matrix4d base_T = baseTransform2d(state(0), state(1), state(2));
    const Eigen::Matrix4d mount_T = mountTransform(relative_R, relative_t);
    result.base_T = base_T;
    result.arm_base_T = base_T * mount_T;

    Eigen::Matrix4d chain = result.arm_base_T;
    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        Eigen::Matrix4d joint_T = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d joint_grad = Eigen::Matrix4d::Zero();
        cr10JointTransform(cr10_joint_fixed_[joint], state(3 + joint), joint_T, joint_grad);
        chain = chain * joint_T;
        result.arm_link_T[joint] = chain;
    }
    result.ee_T = chain;
    return result;
}

// ################################
// C++: TopAY alternating-axis link transforms for tracer regression
// ################################
KinematicResult MomaParam::getLinkTransformsTopayAlt(const Eigen::VectorXd& state) const
{
    KinematicResult result;
    result.arm_link_T.resize(dof_num);

    Eigen::Vector3d now_p(state(0), state(1), chassis_height);
    Eigen::Matrix3d now_R;
    now_R << std::cos(state(2)), -std::sin(state(2)), 0.0,
             std::sin(state(2)), std::cos(state(2)), 0.0,
             0.0, 0.0, 1.0;

    result.base_T = Eigen::Matrix4d::Identity();
    result.base_T.block<3, 3>(0, 0) = now_R;
    result.base_T.block<3, 1>(0, 3) = now_p;

    now_p += now_R * relative_t;
    now_R = now_R * relative_R;

    Eigen::Matrix4d chain = Eigen::Matrix4d::Identity();
    chain.block<3, 3>(0, 0) = now_R;
    chain.block<3, 1>(0, 3) = now_p;
    result.arm_base_T = chain;

    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        now_p += now_R.col(2) * colli_length[joint];
        const double joint_angle = state(3 + joint);
        Eigen::Matrix3d dof_R;
        if (joint % 2 == 0)
        {
            dof_R << std::cos(joint_angle), -std::sin(joint_angle), 0.0,
                     std::sin(joint_angle), std::cos(joint_angle), 0.0,
                     0.0, 0.0, 1.0;
        }
        else
        {
            dof_R << std::cos(joint_angle), 0.0, std::sin(joint_angle),
                     0.0, 1.0, 0.0,
                     -std::sin(joint_angle), 0.0, std::cos(joint_angle);
        }
        now_R = now_R * dof_R;
        chain.block<3, 3>(0, 0) = now_R;
        chain.block<3, 1>(0, 3) = now_p;
        result.arm_link_T[joint] = chain;
    }

    result.ee_T = chain;
    return result;
}

// ################################
// C++: CR10 end-effector pose from typed link transforms
// ################################
Eigen::VectorXd MomaParam::getFKPoseCr10(const Eigen::VectorXd& moma_pos) const
{
    const KinematicResult transforms = getLinkTransformsCr10(moma_pos);
    const Eigen::Matrix3d rotation = transforms.ee_T.block<3, 3>(0, 0);

    Eigen::VectorXd fk_pose = Eigen::VectorXd::Zero(9);
    fk_pose.head(3) = transforms.ee_T.block<3, 1>(0, 3);
    fk_pose.segment(3, 3) = rotation.row(0);
    fk_pose.tail(3) = rotation.row(1);
    return fk_pose;
}

// ################################
// C++: CR10 analytic EE-space to whole-body gradient map
// ################################
Eigen::VectorXd MomaParam::getEEGradsCr10(const Eigen::VectorXd& moma_pos,
                                          const Eigen::VectorXd& ee_grad) const
{
    Eigen::VectorXd moma_grads = Eigen::VectorXd::Zero(3 + static_cast<int>(dof_num));

    const Eigen::Matrix4d base_T = baseTransform2d(moma_pos(0), moma_pos(1), moma_pos(2));
    Eigen::Matrix4d dbase_dyaw = Eigen::Matrix4d::Zero();
    dbase_dyaw(0, 0) = -std::sin(moma_pos(2));
    dbase_dyaw(0, 1) = -std::cos(moma_pos(2));
    dbase_dyaw(1, 0) = std::cos(moma_pos(2));
    dbase_dyaw(1, 1) = -std::sin(moma_pos(2));

    const Eigen::Matrix4d mount_T = mountTransform(relative_R, relative_t);
    Eigen::Matrix4d dbase_dx = Eigen::Matrix4d::Zero();
    dbase_dx(0, 3) = 1.0;
    Eigen::Matrix4d dbase_dy = Eigen::Matrix4d::Zero();
    dbase_dy(1, 3) = 1.0;

    std::vector<Eigen::Matrix4d> joint_transform(dof_num);
    std::vector<Eigen::Matrix4d> joint_transform_grad(dof_num);
    std::vector<Eigen::Matrix4d> prefix(dof_num + 1);
    prefix[0] = base_T * mount_T;

    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        cr10JointTransform(cr10_joint_fixed_[joint], moma_pos(3 + joint),
                           joint_transform[joint], joint_transform_grad[joint]);
        prefix[joint + 1] = prefix[joint] * joint_transform[joint];
    }

    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        Eigen::Matrix4d ee_grad_transform = prefix[joint] * joint_transform_grad[joint];
        for (size_t later = joint + 1; later < dof_num; ++later)
        {
            ee_grad_transform = ee_grad_transform * joint_transform[later];
        }
        moma_grads(3 + static_cast<int>(joint)) += eePoseDirectionalGrad(ee_grad_transform, ee_grad);
    }

    Eigen::Matrix4d ee_grad_yaw = dbase_dyaw * mount_T;
    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        ee_grad_yaw = ee_grad_yaw * joint_transform[joint];
    }
    moma_grads(2) += eePoseDirectionalGrad(ee_grad_yaw, ee_grad);

    Eigen::Matrix4d ee_grad_x = dbase_dx * mount_T;
    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        ee_grad_x = ee_grad_x * joint_transform[joint];
    }
    moma_grads(0) += eePoseDirectionalGrad(ee_grad_x, ee_grad);

    Eigen::Matrix4d ee_grad_y = dbase_dy * mount_T;
    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        ee_grad_y = ee_grad_y * joint_transform[joint];
    }
    moma_grads(1) += eePoseDirectionalGrad(ee_grad_y, ee_grad);

    return moma_grads;
}

// ################################
// C++: Directional gradient of a CR10 sphere center position
// ################################
double positionDirectionalGrad(const Eigen::Matrix4d& transform_grad,
                               const Eigen::Vector3d& local_offset,
                               const Eigen::Vector3d& pos_grad)
{
    const Eigen::Vector3d dp = transform_grad.block<3, 3>(0, 0) * local_offset
                               + transform_grad.block<3, 1>(0, 3);
    return pos_grad.dot(dp);
}

Eigen::Matrix4d cr10LinkTransformAt(const KinematicResult& transforms, int link_id, size_t dof_num)
{
    if (link_id <= 0)
    {
        return transforms.base_T;
    }
    if (link_id == 1)
    {
        return transforms.arm_base_T;
    }
    if (link_id >= 2 && link_id < static_cast<int>(2 + dof_num))
    {
        return transforms.arm_link_T[static_cast<size_t>(link_id - 2)];
    }
    return transforms.ee_T;
}

// ################################
// C++: Sample CR10 link segments and AG95 envelope into collision proxies
// ################################
void MomaParam::buildCollisionProxies()
{
    collision_proxies_.clear();
    colli_self_radii_.clear();

    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        const Eigen::Vector3d segment = cr10_joint_fixed_[joint].block<3, 1>(0, 3);
        const double segment_length = segment.norm();
        if (segment_length <= 1e-6)
        {
            continue;
        }

        const int link_id = static_cast<int>(1 + joint);
        const int sphere_count = std::max(
            1,
            static_cast<int>(std::ceil(segment_length
                                       / (sphere_spacing_factor * obstacle_thickness))));

        for (int sample = 0; sample < sphere_count; ++sample)
        {
            CollisionSphere proxy;
            proxy.link_id = link_id;
            proxy.obstacle_radius = obstacle_thickness;
            proxy.self_radius = self_thickness;
            const double frac =
                (static_cast<double>(sample) + 0.5) / static_cast<double>(sphere_count);
            proxy.local_offset = frac * segment;
            collision_proxies_.push_back(proxy);
            colli_self_radii_.push_back(proxy.self_radius);
        }
    }

    const int ag95_link_id = static_cast<int>(2 + dof_num);
    for (const CollisionSphere& ag95 : ag95_spheres)
    {
        CollisionSphere proxy = ag95;
        proxy.link_id = ag95_link_id;
        collision_proxies_.push_back(proxy);
        colli_self_radii_.push_back(proxy.self_radius);
    }

    colli_link_map.resize(static_cast<Eigen::Index>(collision_proxies_.size()));
    for (size_t idx = 0; idx < collision_proxies_.size(); ++idx)
    {
        colli_link_map(static_cast<Eigen::Index>(idx)) = collision_proxies_[idx].link_id;
    }
}

// ################################
// C++: Build CR10 self-collision ignore topology from link ids
// ################################
void MomaParam::buildCollisionIgnoreMatrix()
{
    const size_t sphere_count = collision_proxies_.size();
    collision_matrix.resize(static_cast<Eigen::Index>(sphere_count),
                            static_cast<Eigen::Index>(sphere_count));
    collision_matrix.setConstant(-1);

    const int ag95_link_id = static_cast<int>(2 + dof_num);
    int last_arm_link_id = 0;
    for (const CollisionSphere& proxy : collision_proxies_)
    {
        if (proxy.link_id != ag95_link_id)
        {
            last_arm_link_id = std::max(last_arm_link_id, proxy.link_id);
        }
    }

    auto hasSampledLinkBetween = [&](int link_a, int link_b) -> bool
    {
        const int lo = std::min(link_a, link_b);
        const int hi = std::max(link_a, link_b);
        for (const CollisionSphere& proxy : collision_proxies_)
        {
            if (proxy.link_id > lo && proxy.link_id < hi)
            {
                return true;
            }
        }
        return false;
    };

    for (size_t i = 0; i < sphere_count; ++i)
    {
        for (size_t j = i; j < sphere_count; ++j)
        {
            if (i == j)
            {
                collision_matrix(static_cast<Eigen::Index>(i),
                                 static_cast<Eigen::Index>(j)) = 1;
                continue;
            }

            const int link_i = collision_proxies_[i].link_id;
            const int link_j = collision_proxies_[j].link_id;
            const bool ignore =
                (link_i == link_j)
                || !hasSampledLinkBetween(link_i, link_j)
                || ((link_i == last_arm_link_id && link_j == ag95_link_id)
                    || (link_j == last_arm_link_id && link_i == ag95_link_id));

            const int ignore_value = ignore ? 1 : -1;
            collision_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) =
                ignore_value;
            collision_matrix(static_cast<Eigen::Index>(j), static_cast<Eigen::Index>(i)) =
                ignore_value;
        }
    }
}

// ################################
// C++: CR10 collision sphere centers from typed link transforms
// ################################
std::vector<Eigen::Vector4d> MomaParam::getColliPtsCr10(const Eigen::VectorXd& moma_pos) const
{
    std::vector<Eigen::Vector4d> colli_pts;
    colli_pts.reserve(collision_proxies_.size());

    const KinematicResult transforms = getLinkTransformsCr10(moma_pos);
    for (const CollisionSphere& proxy : collision_proxies_)
    {
        const Eigen::Matrix4d link_T = cr10LinkTransformAt(transforms, proxy.link_id, dof_num);
        const Eigen::Vector4d local = (Eigen::Vector4d() << proxy.local_offset, 1.0).finished();
        Eigen::Vector4d colli_pt;
        colli_pt.head(3) = (link_T * local).head(3);
        colli_pt[3] = proxy.obstacle_radius;
        colli_pts.push_back(colli_pt);
    }
    return colli_pts;
}

// ################################
// C++: CR10 analytic collision-point gradients on the exact transform chain
// ################################
Eigen::VectorXd MomaParam::getColliGradsCr10(
    const Eigen::VectorXd& moma_pos,
    const std::vector<Eigen::Vector3d>& pos_grads) const
{
    Eigen::VectorXd colli_grads = Eigen::VectorXd::Zero(3 + static_cast<int>(dof_num));

    const Eigen::Matrix4d base_T = baseTransform2d(moma_pos(0), moma_pos(1), moma_pos(2));
    Eigen::Matrix4d dbase_dyaw = Eigen::Matrix4d::Zero();
    dbase_dyaw(0, 0) = -std::sin(moma_pos(2));
    dbase_dyaw(0, 1) = -std::cos(moma_pos(2));
    dbase_dyaw(1, 0) = std::cos(moma_pos(2));
    dbase_dyaw(1, 1) = -std::sin(moma_pos(2));
    Eigen::Matrix4d dbase_dx = Eigen::Matrix4d::Zero();
    dbase_dx(0, 3) = 1.0;
    Eigen::Matrix4d dbase_dy = Eigen::Matrix4d::Zero();
    dbase_dy(1, 3) = 1.0;

    const Eigen::Matrix4d mount_T = mountTransform(relative_R, relative_t);
    std::vector<Eigen::Matrix4d> joint_transform(dof_num);
    std::vector<Eigen::Matrix4d> joint_transform_grad(dof_num);
    std::vector<Eigen::Matrix4d> prefix(dof_num + 1);
    prefix[0] = base_T * mount_T;

    for (size_t joint = 0; joint < dof_num; ++joint)
    {
        cr10JointTransform(cr10_joint_fixed_[joint], moma_pos(3 + joint),
                           joint_transform[joint], joint_transform_grad[joint]);
        prefix[joint + 1] = prefix[joint] * joint_transform[joint];
    }

    for (size_t sphere_idx = 0; sphere_idx < collision_proxies_.size(); ++sphere_idx)
    {
        const CollisionSphere& proxy = collision_proxies_[sphere_idx];
        const Eigen::Vector3d pos_grad = pos_grads[sphere_idx];
        const int link_id = proxy.link_id;
        const int last_joint_index =
            link_id >= 2 ? std::min(static_cast<int>(dof_num) - 1, link_id - 2) : -1;

        for (int joint = 0; joint <= last_joint_index; ++joint)
        {
            Eigen::Matrix4d grad_transform = prefix[static_cast<size_t>(joint)]
                                             * joint_transform_grad[static_cast<size_t>(joint)];
            for (int later = joint + 1; later <= last_joint_index; ++later)
            {
                grad_transform = grad_transform * joint_transform[static_cast<size_t>(later)];
            }
            colli_grads(3 + joint) += positionDirectionalGrad(
                grad_transform, proxy.local_offset, pos_grad);
        }

        if (link_id >= 1)
        {
            Eigen::Matrix4d grad_yaw = dbase_dyaw * mount_T;
            Eigen::Matrix4d grad_x = dbase_dx * mount_T;
            Eigen::Matrix4d grad_y = dbase_dy * mount_T;
            for (int joint = 0; joint <= last_joint_index; ++joint)
            {
                grad_yaw = grad_yaw * joint_transform[static_cast<size_t>(joint)];
                grad_x = grad_x * joint_transform[static_cast<size_t>(joint)];
                grad_y = grad_y * joint_transform[static_cast<size_t>(joint)];
            }
            colli_grads(2) += positionDirectionalGrad(grad_yaw, proxy.local_offset, pos_grad);
            colli_grads(0) += positionDirectionalGrad(grad_x, proxy.local_offset, pos_grad);
            colli_grads(1) += positionDirectionalGrad(grad_y, proxy.local_offset, pos_grad);
        }
    }

    return colli_grads;
}

// ################################
// C++: Dual-radius lookup for self-collision consumers
// ################################
double MomaParam::getColliSelfRadius(size_t idx) const
{
    if (idx < colli_self_radii_.size())
    {
        return colli_self_radii_[idx];
    }
    return 0.0;
}

bool MomaParam::isChassisArmCollisionIgnored(int link_id) const
{
    for (int ignored_link : chassis_ignore_link_ids)
    {
        if (ignored_link == link_id)
        {
            return true;
        }
    }
    return false;
}
