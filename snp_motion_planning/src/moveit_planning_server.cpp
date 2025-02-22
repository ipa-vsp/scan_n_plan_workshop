#include <rclcpp/rclcpp.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <sensor_msgs/msg/joint_state.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometric_shapes/shapes.h> 
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/srv/generate_freespace_motion_plan.hpp>
#include <snp_msgs/msg/tool_path.hpp>
#include <std_srvs/srv/empty.hpp>
// #include <tesseract_common/eigen_types.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "snp_motion_planning/cartesian_planner.hpp"

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "SNPD";
static const std::string SCAN_LINK_NAME = "scan";

// Parameters
//   General
static const std::string SCAN_DISABLED_CONTACT_LINKS = "scan_disabled_contact_links";
static const std::string SCAN_REDUCED_CONTACT_LINKS_PARAM = "scan_reduced_contact_links";
static const std::string VERBOSE_PARAM = "verbose";
//   Scan link
static const std::string COLLISION_OBJECT_TYPE_PARAM = "collision_object_type";
static const std::string OCTREE_RESOLUTION_PARAM = "octree_resolution";
//   Task composer
static const std::string TASK_COMPOSER_CONFIG_FILE_PARAM = "task_composer_config_file";
static const std::string RASTER_TASK_NAME_PARAM = "raster_task_name";
static const std::string FREESPACE_TASK_NAME_PARAM = "freespace_task_name";

//   Profile
static const std::string MAX_TRANS_VEL_PARAM = "max_translational_vel";
static const std::string MAX_ROT_VEL_PARAM = "max_rotational_vel";
static const std::string MAX_TRANS_ACC_PARAM = "max_translational_acc";
static const std::string MAX_ROT_ACC_PARAM = "max_rotational_acc";
static const std::string CHECK_JOINT_ACC_PARAM = "check_joint_accelerations";
static const std::string VEL_SCALE_PARAM = "velocity_scaling_factor";
static const std::string ACC_SCALE_PARAM = "acceleration_scaling_factor";
static const std::string LVS_PARAM = "contact_check_lvs_distance";
static const std::string MIN_CONTACT_DIST_PARAM = "min_contact_distance";
static const std::string OMPL_MAX_PLANNING_TIME_PARAM = "ompl_max_planning_time";
static const std::string TCP_MAX_SPEED_PARAM = "tcp_max_speed";
static const std::string TRAJOPT_CARTESIAN_TOLERANCE_PARAM = "cartesian_tolerance";
static const std::string TRAJOPT_CARTESIAN_COEFFICIENT_PARAM = "cartesian_coefficient";

// Topics
static const std::string TESSERACT_MONITOR_NAMESPACE = "snp_environment";

// Services
static const std::string PLANNING_SERVICE = "generate_motion_plan";
static const std::string FREESPACE_PLANNING_SERVICE = "generate_freespace_motion_plan";
static const std::string REMOVE_SCAN_LINK_SERVICE = "remove_scan_link";

// tesseract_common::Toolpath fromMsg(const std::vector<snp_msgs::msg::ToolPath>& paths)
// {
//   tesseract_common::Toolpath tps;
//   tps.reserve(paths.size());
//   for (const auto& path : paths)
//   {
//     for (const auto& segment : path.segments)
//     {
//       tesseract_common::VectorIsometry3d seg;
//       seg.reserve(segment.poses.size());
//       for (const auto& pose : segment.poses)
//       {
//         Eigen::Isometry3d p;
//         tf2::fromMsg(pose, p);
//         // // Rotate the pose 180 degrees about the x-axis such that the z-axis faces into the part
//         // p *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
//         seg.push_back(p);
//       }
//       tps.push_back(seg);
//     }
//   }
//   return tps;
// }

class MoveItPlanningServer
{
    public:
        MoveItPlanningServer(rclcpp::Node::SharedPtr node): node_(node)
        {
            // Declare ROS parameters
            // node_->declare_parameter(VERBOSE_PARAM, false);
            // node_->declare_parameter<std::vector<std::string>>(SCAN_DISABLED_CONTACT_LINKS, {});
            // node_->declare_parameter<std::vector<std::string>>(SCAN_REDUCED_CONTACT_LINKS_PARAM, {});
            // node_->declare_parameter(OCTREE_RESOLUTION_PARAM, 0.010);
            // node_->declare_parameter(COLLISION_OBJECT_TYPE_PARAM, "convex_mesh");

            // // Profiles
            // node_->declare_parameter(MAX_TRANS_VEL_PARAM, 0.05);
            // node_->declare_parameter(MAX_ROT_VEL_PARAM, 1.571);
            // node_->declare_parameter(MAX_TRANS_ACC_PARAM, 0.1);
            // node_->declare_parameter(MAX_ROT_ACC_PARAM, 3.14159);
            // node_->declare_parameter<bool>(CHECK_JOINT_ACC_PARAM, false);
            // node_->declare_parameter<double>(VEL_SCALE_PARAM, 1.0);
            // node_->declare_parameter<double>(ACC_SCALE_PARAM, 1.0);
            // node_->declare_parameter<double>(LVS_PARAM, 0.05);
            // node_->declare_parameter<double>(MIN_CONTACT_DIST_PARAM, 0.0);
            // node_->declare_parameter<double>(OMPL_MAX_PLANNING_TIME_PARAM, 5.0);
            // node_->declare_parameter<double>(TCP_MAX_SPEED_PARAM, 0.25);
            // node_->declare_parameter<std::vector<double>>(TRAJOPT_CARTESIAN_TOLERANCE_PARAM, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            // node_->declare_parameter<std::vector<double>>(TRAJOPT_CARTESIAN_COEFFICIENT_PARAM,
            //                                             { 2.5, 2.5, 2.5, 2.5, 2.5, 0.0 });

            // // Task composer
            // node_->declare_parameter(TASK_COMPOSER_CONFIG_FILE_PARAM, "");
            // node_->declare_parameter(RASTER_TASK_NAME_PARAM, "");
            // node_->declare_parameter(FREESPACE_TASK_NAME_PARAM, "");

            // Create MoveItCpp
            moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
            moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
            moveit_cpp_->getPlanningSceneMonitor()->startSceneMonitor();
            moveit_cpp_->getPlanningSceneMonitor()->startStateMonitor();
            moveit_cpp_->getPlanningSceneMonitor()->startWorldGeometryMonitor();

            psm_ = moveit_cpp_->getPlanningSceneMonitor();


            freespace_server_ = node_->create_service<snp_msgs::srv::GenerateFreespaceMotionPlan>(
                FREESPACE_PLANNING_SERVICE, std::bind(&MoveItPlanningServer::processFreespaceMotionPlanCallback, this, std::placeholders::_1, std::placeholders::_2));
            raster_server_ = node_->create_service<snp_msgs::srv::GenerateMotionPlan>(
                PLANNING_SERVICE, std::bind(&MoveItPlanningServer::processMotionPlanCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
    
    void processFreespaceMotionPlanCallback (const snp_msgs::srv::GenerateFreespaceMotionPlan::Request::SharedPtr req,
                                   snp_msgs::srv::GenerateFreespaceMotionPlan::Response::SharedPtr res)
    {
        std::string motion_group = req->motion_group;
        std::string tcp_frame = req->tcp_frame;
        sensor_msgs::msg::JointState js1 = req->js1;
        sensor_msgs::msg::JointState js2 = req->js2;
        std::string mesh_filename = req->mesh_filename;
        std::string mesh_frame = req->mesh_frame;

        RCLCPP_INFO(node_->get_logger(), "Received freespace motion planning request for group: %s", motion_group.c_str());

        auto planning_component = std::make_shared<moveit_cpp::PlanningComponent>(motion_group, moveit_cpp_);
        auto robot_model = moveit_cpp_->getRobotModel();
        auto robot_start_state = planning_component->getStartState();
        auto joint_model_group_ptr = robot_model->getJointModelGroup(motion_group);

        // if(!req->mesh_filename.empty())
        // {
        //     shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_filename);
        //     if(mesh)
        //     {
        //         moveit_msgs::msg::CollisionObject collision_object;
        //         collision_object.id = "scanned_mesh";
        //         collision_object.header.frame_id = mesh_frame;
        //         shape_msgs::msg::Mesh mesh_msg;
        //         shapes::ShapeMsg shape_msg;
        //         shapes::constructMsgFromShape(mesh, shape_msg);
        //         mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
        //         collision_object.meshes.push_back(mesh_msg);
        //         collision_object.mesh_poses.push_back(geometry_msgs::msg::Pose());
        //         collision_object.operation = collision_object.ADD;

        //         {
        //             planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
        //             ps->processCollisionObjectMsg(collision_object);
        //         }
        //     }
        //     else
        //     {
        //         RCLCPP_ERROR(node_->get_logger(), "Failed to load mesh: %s", req->mesh_filename.c_str());
        //         res->success = false;
        //         res->message = "Failed to load mesh";
        //         return;
        //     }
        // }

        planning_component->setStartStateToCurrentState();
        moveit::core::RobotState goal_state(robot_model);
        goal_state.setJointGroupPositions(joint_model_group_ptr, js2.position);
        planning_component->setGoal(goal_state);

        auto plan_solution = planning_component->plan();
        if(plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan motion: %d", plan_solution.error_code.val);
            res->success = false;
            res->message = "Failed to plan motion";
            return;
        }

        moveit_msgs::msg::RobotTrajectory traj_msg;
        plan_solution.trajectory->getRobotTrajectoryMsg(traj_msg, {});
        res->trajectory = traj_msg.joint_trajectory;
        res->success = true;
        res->message = "Succesfully planned motion";
        RCLCPP_INFO(node_->get_logger(), "Succesfully planned motion");
    }

    void processMotionPlanCallback (const snp_msgs::srv::GenerateMotionPlan::Request::SharedPtr req,
                                    snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr res)
    {
        try
        {
            RCLCPP_INFO(node_->get_logger(), "Received motion planning request");
            if(req->motion_group.empty())
                throw std::runtime_error("Motion group is empty");
            if(req->tcp_frame.empty())
                throw std::runtime_error("TCP frame is empty");
            if(req->tool_paths.empty())
                throw std::runtime_error("Tool paths are empty");

            auto planning_component = std::make_shared<moveit_cpp::PlanningComponent>(req->motion_group, moveit_cpp_);
            auto robot_model = moveit_cpp_->getRobotModel();
            auto joint_model_group_ptr = robot_model->getJointModelGroup(req->motion_group);

            if (!joint_model_group_ptr)
                throw std::runtime_error("Invalid JointModelGroup for motion group: " + req->motion_group);

            planning_component->setStartStateToCurrentState();
            moveit::core::RobotState start_state(*moveit_cpp_->getCurrentState());

            EigenSTL::vector_Isometry3d waypoints;
            for (const auto& pose : req->tool_paths)
            {
                for (const auto& segment : pose.segments)
                {
                    Eigen::Isometry3d eigen_pose;
                    tf2::fromMsg(segment.poses[0], eigen_pose);
                    waypoints.push_back(eigen_pose);
                }
            }

            if (waypoints.empty())
                throw std::runtime_error("No valid waypoints extracted from tool paths");

            MaxEEFStep max_eef_step(0.01, 0.1);
            CartesianPrecision cartesian_precision{.translational = 0.001,
                                                    .rotational = 0.01,
                                                    .max_resolution = 1e-3};

            std::vector<moveit::core::RobotStatePtr> traj;
            kinematics::KinematicsQueryOptions opts;
            opts.return_approximate_solution = true;

            // Cost function
            const double weight = 0.0005;
            const auto compute_l2_norm = [](const std::vector<double>& solution, const std::vector<double>& start) {
                double sum = 0.0;
                for (size_t ji = 0; ji < solution.size(); ji++)
                {
                    double d = solution[ji] - start[ji];
                    sum += d * d;
                }
                return sum;
            };

            // const auto cost_fn = [&weight, &compute_l2_norm](const geometry_msgs::msg::Pose& /*goal_pose*/,
            //                                 const moveit::core::RobotState& solution_state,
            //                                 const moveit::core::JointModelGroup* jmg,
            //                                 const std::vector<double>& seed_state) {
            //     std::vector<double> proposed_joint_positions;
            //     solution_state.copyJointGroupPositions(jmg, proposed_joint_positions);
            //     double cost = compute_l2_norm(proposed_joint_positions, seed_state);
            //     return weight * cost;
            // };

            auto cost_fn = kinematics::KinematicsBase::IKCostFn();

            CartesianPathPlanner planner;
            Eigen::Isometry3d link_offset = Eigen::Isometry3d::Identity();

            // Compute Cartesian path
            RCLCPP_INFO(node_->get_logger(), "Computing Cartesian path");
            Percentage per = planner.computeCartesianPath(&start_state, joint_model_group_ptr, traj,
                                        robot_model->getLinkModel(req->tcp_frame), waypoints, true,
                                        max_eef_step, cartesian_precision, callback_fn_,
                                        opts, cost_fn, link_offset);

            if (per < 0.85)
                throw std::runtime_error("Cartesian path planning only completed " + std::to_string(per.value * 100) + "% of the path");

            std::cout << "Cartesian path planning completed " << per.value * 100 << "% of the path" << std::endl;
            std::cout << "Number of waypoints: " << traj.size() << std::endl;
    
            if (traj.empty())
                throw std::runtime_error("Generated trajectory is empty.");
            
            moveit_msgs::msg::RobotTrajectory traj_msg;
            traj_msg.joint_trajectory.header.stamp = rclcpp::Clock().now();
            traj_msg.joint_trajectory.joint_names = joint_model_group_ptr->getVariableNames();

            for (int i = 0; i < traj.size(); i++)
            {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                std::vector<double> joint_values;
                traj[i]->copyJointGroupPositions(joint_model_group_ptr, joint_values);
                point.positions = joint_values;
                point.time_from_start = rclcpp::Duration::from_seconds(0.1 * i); 
                traj_msg.joint_trajectory.points.push_back(point);
            }
            std::cout << "Generated trajectory with " << traj_msg.joint_trajectory.points.size() << " points" << std::endl;
            // Split trajectory into approach, process, and departure
            size_t total_points = traj_msg.joint_trajectory.points.size();
            if (total_points < 3)
                throw std::runtime_error("Trajectory does not have enough points to segment into approach, process, and departure.");

            size_t approach_end = total_points / 3;
            size_t departure_start = (2 * total_points) / 3;

            // Assign first third to approach
            res->approach.joint_names = traj_msg.joint_trajectory.joint_names;
            res->approach.points.assign(traj_msg.joint_trajectory.points.begin(),
                                        traj_msg.joint_trajectory.points.begin() + approach_end);

            // Assign middle third to process
            res->process.joint_names = traj_msg.joint_trajectory.joint_names;
            res->process.points.assign(traj_msg.joint_trajectory.points.begin() + approach_end,
                                    traj_msg.joint_trajectory.points.begin() + departure_start);

            // Assign last third to departure
            res->departure.joint_names = traj_msg.joint_trajectory.joint_names;
            res->departure.points.assign(traj_msg.joint_trajectory.points.begin() + departure_start,
                                        traj_msg.joint_trajectory.points.end());

            res->success = true;
            res->message = "Cartesian path planning succeeded";

            RCLCPP_INFO(node_->get_logger(), "Motion planning succeeded.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Motion planning failed: %s", e.what());
            res->success = false;
        }
    }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<snp_msgs::srv::GenerateMotionPlan>::SharedPtr raster_server_;
        rclcpp::Service<snp_msgs::srv::GenerateFreespaceMotionPlan>::SharedPtr freespace_server_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr remove_scan_link_server_;
        moveit_cpp::MoveItCppPtr moveit_cpp_;
        planning_scene_monitor::PlanningSceneMonitorPtr psm_;
        GroupStateValidityCallbackFn callback_fn_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("snp_moveit_planning_server", options);
    auto server = std::make_shared<MoveItPlanningServer>(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}