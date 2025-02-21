#ifndef CARTESIAN_PATH_PLANNER_HPP
#define CARTESIAN_PATH_PLANNER_HPP

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene/planning_scene.h>
#include <Eigen/Geometry>
#include <moveit/macros/declare_ptr.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/utils/message_checks.h>


struct MaxEEFStep
{
  MaxEEFStep(double translation, double rotation) : translation(translation), rotation(rotation)
  {
  }

  MaxEEFStep(double step_size) : translation(step_size), rotation(3.5 * step_size)  // 0.035 rad = 2 deg
  {
  }

  double translation;  // Meters
  double rotation;     // Radians
};

struct Percentage
{
    // value must be in [0,1]
    Percentage(double value) : value(value)
    {
        if (value < 0.0 || value > 1.0)
            throw std::runtime_error("Percentage values must be between 0 and 1, inclusive");
    }
    operator double() { return value; }
    double operator*() { return value; }
    Percentage operator*(const Percentage& p) { return Percentage(value * p.value); }
    double value;
};

struct Distance
{
    Distance(double meters) : meters(meters) {}
    operator double() { return meters; }
    double operator*() { return meters; }
    Distance operator*(const Percentage& p) { return Distance(meters * p.value); }
    double meters;
};

struct CartesianPrecision
{
    double translational = 0.001;  //< max deviation in translation (meters)
    double rotational = 0.01;      //< max deviation in rotation (radians)
    double max_resolution = 1e-5;  //< max resolution for waypoints (fraction of total path)
};

typedef std::function<bool(moveit::core::RobotState* robot_state, const moveit::core::JointModelGroup* joint_group,
    const double* joint_group_variable_values)> GroupStateValidityCallbackFn;

class CartesianPathPlanner
{
public:
    Distance computeCartesianPath(
        const moveit::core::RobotState* start_state, const moveit::core::JointModelGroup* group, std::vector<moveit::core::RobotStatePtr>& traj,
        const moveit::core::LinkModel* link, const Eigen::Vector3d& translation, bool global_reference_frame, const MaxEEFStep& max_step,
        const CartesianPrecision& precision, const GroupStateValidityCallbackFn& validCallback,
        const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function)
    {
        const double distance = translation.norm();
        // The target pose is obtained by adding the translation vector to the link's current pose
        Eigen::Isometry3d pose = start_state->getGlobalLinkTransform(link);

        // the translation direction can be specified w.r.t. the local link frame (then rotate into global frame)
        pose.translation() += global_reference_frame ? translation : pose.linear() * translation;

        // call computeCartesianPath for the computed target pose in the global reference frame
        return Distance(distance) * this->computeCartesianPath(start_state, group, traj, link, pose, true,
                                                                                max_step, precision, validCallback, options,
                                                                                cost_function);
    }

    Percentage computeCartesianPath(
        const moveit::core::RobotState* start_state, const moveit::core::JointModelGroup* group, std::vector<moveit::core::RobotStatePtr>& traj,
        const moveit::core::LinkModel* link, const Eigen::Isometry3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
        const CartesianPrecision& precision, const GroupStateValidityCallbackFn& validCallback,
        const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function,
        const Eigen::Isometry3d& link_offset)
    {
        // check unsanitized inputs for non-isometry
        ASSERT_ISOMETRY(target)
        ASSERT_ISOMETRY(link_offset)

        moveit::core::RobotState state(*start_state);

        const std::vector<const moveit::core::JointModel*>& cjnt = group->getContinuousJointModels();
        // make sure that continuous joints wrap
        for (const moveit::core::JointModel* joint : cjnt)
            state.enforceBounds(joint);

        // Cartesian pose we start from
        Eigen::Isometry3d start_pose = state.getGlobalLinkTransform(link) * link_offset;
        Eigen::Isometry3d inv_offset = link_offset.inverse();

        // the target can be in the local reference frame (in which case we rotate it)
        Eigen::Isometry3d rotated_target = global_reference_frame ? target : start_pose * target;

        Eigen::Quaterniond start_quaternion(start_pose.linear());
        Eigen::Quaterniond target_quaternion(rotated_target.linear());

        double rotation_distance = start_quaternion.angularDistance(target_quaternion);
        double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

        // decide how many steps we will need for this trajectory
        std::size_t translation_steps = 0;
        if (max_step.translation > 0.0)
            translation_steps = floor(translation_distance / max_step.translation);

        std::size_t rotation_steps = 0;
        if (max_step.rotation > 0.0)
            rotation_steps = floor(rotation_distance / max_step.rotation);
        std::size_t steps = std::max(translation_steps, rotation_steps) + 1;

        traj.clear();
        traj.push_back(std::make_shared<moveit::core::RobotState>(*start_state));

        double last_valid_percentage = 0.0;
        Eigen::Isometry3d prev_pose = start_pose;
        moveit::core::RobotState prev_state(state);
        for (std::size_t i = 1; i <= steps; ++i)
        {
            double percentage = static_cast<double>(i) / static_cast<double>(steps);

            Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
            pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

            if (!state.setFromIK(group, pose * inv_offset, link->getName(), 0.0, validCallback, options, cost_function) ||
                !validateAndImproveInterval(prev_state, state, prev_pose, pose, traj, percentage,
                                            1.0 / static_cast<double>(steps), group, link, precision, validCallback, options,
                                            cost_function, link_offset))
            break;

            prev_pose = pose;
            prev_state = state;
            last_valid_percentage = percentage;
        }

        return last_valid_percentage;
    }

    Percentage computeCartesianPath(
        const moveit::core::RobotState* start_state, const moveit::core::JointModelGroup* group, std::vector<moveit::core::RobotStatePtr>& traj,
        const moveit::core::LinkModel* link, const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame,
        const MaxEEFStep& max_step, const CartesianPrecision& precision, const GroupStateValidityCallbackFn& validCallback,
        const kinematics::KinematicsQueryOptions& options, const kinematics::KinematicsBase::IKCostFn& cost_function,
        const Eigen::Isometry3d& link_offset)
    {
        double percentage_solved = 0.0;
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        {
            std::vector<moveit::core::RobotStatePtr> waypoint_traj;
            double wp_percentage_solved =
                computeCartesianPath(start_state, group, waypoint_traj, link, waypoints[i], global_reference_frame, max_step,
                                    precision, validCallback, options, cost_function, link_offset);

            std::vector<moveit::core::RobotStatePtr>::iterator start = waypoint_traj.begin();
            if (i > 0 && !waypoint_traj.empty())
            std::advance(start, 1);
            traj.insert(traj.end(), start, waypoint_traj.end());

            if (fabs(wp_percentage_solved - 1.0) < std::numeric_limits<double>::epsilon())
            {
            percentage_solved = static_cast<double>(i + 1) / static_cast<double>(waypoints.size());
            }
            else
            {
            percentage_solved += wp_percentage_solved / static_cast<double>(waypoints.size());
            break;
            }
            start_state = traj.back().get();
        }

        return percentage_solved;
    }


private:
    bool validateAndImproveInterval(const moveit::core::RobotState& start_state, const moveit::core::RobotState& end_state,
        const Eigen::Isometry3d& start_pose, const Eigen::Isometry3d& end_pose,
        std::vector<moveit::core::RobotStatePtr>& traj, double& percentage, const double width,
        const moveit::core::JointModelGroup* group, const moveit::core::LinkModel* link,
        const CartesianPrecision& precision, const GroupStateValidityCallbackFn& validCallback,
        const kinematics::KinematicsQueryOptions& options,
        const kinematics::KinematicsBase::IKCostFn& cost_function,
        const Eigen::Isometry3d& link_offset)
    {
        // compute pose at joint-space midpoint between start_state and end_state
        moveit::core::RobotState mid_state(start_state.getRobotModel());
        start_state.interpolate(end_state, 0.5, mid_state);
        Eigen::Isometry3d fk_pose = mid_state.getGlobalLinkTransform(link) * link_offset;

        // compute pose at Cartesian-space midpoint between start_pose and end_pose
        Eigen::Isometry3d mid_pose(Eigen::Quaterniond(start_pose.linear()).slerp(0.5, Eigen::Quaterniond(end_pose.linear())));
        mid_pose.translation() = 0.5 * (start_pose.translation() + end_pose.translation());

        // if deviation between both poses, fk_pose and mid_pose is within precision, we are satisfied
        double linear_distance = (mid_pose.translation() - fk_pose.translation()).norm();
        double angular_distance = Eigen::Quaterniond(mid_pose.linear()).angularDistance(Eigen::Quaterniond(fk_pose.linear()));
        if (linear_distance <= precision.translational && angular_distance <= precision.rotational)
        {
        traj.push_back(std::make_shared<moveit::core::RobotState>(end_state));
        return true;
        }

        if (width < precision.max_resolution)
        return false;  // failed to find linear interpolation within max_resolution

        // otherwise subdivide interval further, computing IK for mid_pose
        if (!mid_state.setFromIK(group, mid_pose * link_offset.inverse(), link->getName(), 0.0, validCallback, options,
        cost_function))
        return false;

        // and recursively processing the two sub-intervals
        const auto half_width = width / 2.0;
        const auto old_percentage = percentage;
        percentage = percentage - half_width;
        if (!validateAndImproveInterval(start_state, mid_state, start_pose, mid_pose, traj, percentage, half_width, group,
            link, precision, validCallback, options, cost_function, link_offset))
        return false;

        percentage = old_percentage;
        return validateAndImproveInterval(mid_state, end_state, mid_pose, end_pose, traj, percentage, half_width, group, link,
                precision, validCallback, options, cost_function, link_offset);
    }
};

#endif // CARTESIAN_PATH_PLANNER_HPP
