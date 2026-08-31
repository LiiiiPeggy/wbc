#include "fake_moma/visual_transform_utils.h"

#include <stdexcept>

namespace fake_moma_visual
{

// ################################
// C++: Select the planning-frame link transform for a profile mesh role
// ################################
Eigen::Matrix4d meshPlanningLinkTransform(const KinematicResult& links, const MeshPart& part)
{
    switch (part.role)
    {
        case MeshRole::Base:
            return links.base_T;
        case MeshRole::ArmBase:
            return links.arm_base_T;
        case MeshRole::ArmLink:
            return links.arm_link_T.at(part.index);
        case MeshRole::Ag95:
            return links.ee_T;
    }
    throw std::runtime_error("Unknown profile mesh role in meshPlanningLinkTransform");
}

// ################################
// C++: World CAD pose = applyVisualRoot(planning_link) * link_T_visual
// ################################
Eigen::Matrix4d meshWorldVisualTransform(const MomaParam& param,
                                         const KinematicResult& links,
                                         const MeshPart& part)
{
    const Eigen::Matrix4d planning_link = meshPlanningLinkTransform(links, part);
    return param.applyVisualRoot(links.base_T, planning_link) * part.link_T_visual;
}

geometry_msgs::Pose poseFromTransform(const Eigen::Matrix4d& transform)
{
    geometry_msgs::Pose pose;
    const Eigen::Quaterniond orientation(transform.block<3, 3>(0, 0));
    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = transform(2, 3);
    pose.orientation.w = orientation.w();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    return pose;
}

}  // namespace fake_moma_visual
