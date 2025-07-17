from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder(robot_name="abb_arm")
        .robot_description_semantic(file_path="/home/brian_2025/Github/moveit_ws/install/abb_arm_moveit_config/share/abb_arm_moveit_config/config/abb_arm.srdf", mappings={"name": "abb_arm"})
        .robot_description_kinematics(file_path="/home/brian_2025/Github/moveit_ws/install/abb_arm_description/share/abb_arm_description/config/kinematics.yaml")
        .robot_description(file_path="/home/brian_2025/Github/moveit_ws/install/abb_platform/share/abb_platform/urdf/robot.urdf.xacro", mappings={"name": "abb_arm"})
        .joint_limits(file_path="/home/brian_2025/Github/moveit_ws/install/abb_arm_moveit_config/share/abb_arm_moveit_config/config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="/home/brian_2025/Github/moveit_ws/install/abb_arm_moveit_config/share/abb_arm_moveit_config/config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="/home/brian_2025/Github/moveit_ws/install/abb_arm_moveit_config/share/abb_arm_moveit_config/config/ros2_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_dict()
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "state_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("abb_arm_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Polish Demo node
    polish_demo = Node(
        package="abb_mtc",
        executable="abb_mtc",
        output="screen",
        parameters=[
            ompl_planning_pipeline_config,
            moveit_config,
        ],
    )

    return LaunchDescription([polish_demo])