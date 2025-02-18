import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument("verbose", default_value="False"),
        DeclareLaunchArgument("scan_disabled_contact_links", default_value="['']"),
        DeclareLaunchArgument("scan_reduced_contact_links", default_value="['']"),
        DeclareLaunchArgument("collision_object_type", default_value="convex_mesh"),
        DeclareLaunchArgument("octree_resolution", default_value="0.010"),
        DeclareLaunchArgument("task_composer_config_file", 
            default_value=os.path.join(get_package_share_directory("snp_motion_planning"), "config", "task_composer_plugins.yaml")
        ),
        DeclareLaunchArgument("moveit_cpp_plugin_config", 
            default_value=os.path.join(get_package_share_directory("snp_motion_planning"), "config", "moveitcpp.yaml")
        ),
        DeclareLaunchArgument("raster_task_name", default_value="SNPPipeline"),
        DeclareLaunchArgument("freespace_task_name", default_value="SNPFreespacePipeline"),
        DeclareLaunchArgument("velocity_scaling_factor", default_value="1.0"),
        DeclareLaunchArgument("acceleration_scaling_factor", default_value="1.0"),
        DeclareLaunchArgument("max_translational_vel", default_value="0.050"),
        DeclareLaunchArgument("max_rotational_vel", default_value="1.571"),
        DeclareLaunchArgument("max_translational_acc", default_value="0.100"),
        DeclareLaunchArgument("max_rotational_acc", default_value="3.14"),
        DeclareLaunchArgument("check_joint_accelerations", default_value="false"),
        DeclareLaunchArgument("min_contact_distance", default_value="0.0"),
        DeclareLaunchArgument("contact_check_lvs_distance", default_value="0.05"),
        DeclareLaunchArgument("ompl_max_planning_time", default_value="5.0"),
        DeclareLaunchArgument("tcp_max_speed", default_value="0.25"),
        DeclareLaunchArgument("cartesian_tolerance", default_value="[0.01, 0.01, 0.01, 0.05, 0.05, 6.28]"),
        DeclareLaunchArgument("cartesian_coefficient", default_value="[2.5, 2.5, 2.5, 2.5, 2.5, 0.0]")
    ]
    
    moveit_config = (
        MoveItConfigsBuilder("rox", package_name="erf_demo_moveit_config")
        .robot_description(
            file_path=os.path.join(get_package_share_directory("erf_demo_description"), "urdf", "erf.urdf.xacro")
        )
        .moveit_cpp(
            file_path=os.path.join(get_package_share_directory("snp_motion_planning"), "config", "moveitcpp.yaml")
        )
        .to_moveit_configs()
    )
    
    # Node definition
    snp_motion_planning_node = Node(
        package="snp_motion_planning",
        executable="snp_motion_planning_moveit_node",
        output="screen",
        parameters=[
            # {"verbose": LaunchConfiguration("verbose")},
            # {"scan_disabled_contact_links": LaunchConfiguration("scan_disabled_contact_links")},
            # {"scan_reduced_contact_links": LaunchConfiguration("scan_reduced_contact_links")},
            # {"collision_object_type": LaunchConfiguration("collision_object_type")},
            # {"octree_resolution": LaunchConfiguration("octree_resolution")},
            # {"task_composer_config_file": LaunchConfiguration("task_composer_config_file")},
            # {"raster_task_name": LaunchConfiguration("raster_task_name")},
            # {"freespace_task_name": LaunchConfiguration("freespace_task_name")},
            # {"velocity_scaling_factor": LaunchConfiguration("velocity_scaling_factor")},
            # {"acceleration_scaling_factor": LaunchConfiguration("acceleration_scaling_factor")},
            # {"max_translational_vel": LaunchConfiguration("max_translational_vel")},
            # {"max_rotational_vel": LaunchConfiguration("max_rotational_vel")},
            # {"max_translational_acc": LaunchConfiguration("max_translational_acc")},
            # {"max_rotational_acc": LaunchConfiguration("max_rotational_acc")},
            # {"check_joint_accelerations": LaunchConfiguration("check_joint_accelerations")},
            # {"min_contact_distance": LaunchConfiguration("min_contact_distance")},
            # {"contact_check_lvs_distance": LaunchConfiguration("contact_check_lvs_distance")},
            # {"ompl_max_planning_time": LaunchConfiguration("ompl_max_planning_time")},
            # {"tcp_max_speed": LaunchConfiguration("tcp_max_speed")},
            # {"cartesian_tolerance": LaunchConfiguration("cartesian_tolerance")},
            # {"cartesian_coefficient": LaunchConfiguration("cartesian_coefficient")},
            moveit_config.to_dict(),
        ]
    )

    return LaunchDescription(launch_arguments + [snp_motion_planning_node])
