#pragma once

#include "fake_moma/moma_param.h"
#include <geometry_msgs/Pose.h>

// ################################
// C++: Shared planning-link -> world-visual mesh pose helpers for sim/vis/planner
// ################################
namespace fake_moma_visual
{

Eigen::Matrix4d meshPlanningLinkTransform(const KinematicResult& links, const MeshPart& part);

Eigen::Matrix4d meshWorldVisualTransform(const MomaParam& param,
                                         const KinematicResult& links,
                                         const MeshPart& part);

geometry_msgs::Pose poseFromTransform(const Eigen::Matrix4d& transform);

}  // namespace fake_moma_visual
