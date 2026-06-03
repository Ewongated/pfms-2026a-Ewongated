from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the racing_track_node with configurable parameters.

    Usage (P/C):
        ros2 launch racing_track_pkg racing_track.launch.py

    Usage (D/HD — laser-derived waypoints):
        ros2 launch racing_track_pkg racing_track.launch.py advanced:=true

    Parameters:
        advanced          (bool,   default false) -- D/HD: use laser-derived waypoints
        track_width       (double, default 8.0)   -- nominal track width [m]
        goal_tolerance    (double, default 0.2)   -- goal-in-corridor lateral tolerance [m]
        obstacle_range    (double, default 6.0)   -- forward obstacle detection distance [m]
        waypoint_spacing  (double, default 3.0)   -- waypoint spacing for D/HD mode [m]
        goal_tolerance_reached (double, default 0.5) -- distance to declare goal reached [m]
    """
    return LaunchDescription([
        DeclareLaunchArgument("advanced",               default_value="false"),
        DeclareLaunchArgument("track_width",            default_value="8.0"),
        DeclareLaunchArgument("goal_tolerance",         default_value="0.2"),
        DeclareLaunchArgument("obstacle_range",         default_value="6.0"),
        DeclareLaunchArgument("waypoint_spacing",       default_value="3.0"),
        DeclareLaunchArgument("goal_tolerance_reached", default_value="0.5"),

        Node(
            package    = "racing_track_pkg",
            executable = "racing_track_node",
            name       = "racing_track_node",
            output     = "screen",
            parameters = [{
                "advanced":               LaunchConfiguration("advanced"),
                "track_width":            LaunchConfiguration("track_width"),
                "goal_tolerance":         LaunchConfiguration("goal_tolerance"),
                "obstacle_range":         LaunchConfiguration("obstacle_range"),
                "waypoint_spacing":       LaunchConfiguration("waypoint_spacing"),
                "goal_tolerance_reached": LaunchConfiguration("goal_tolerance_reached"),
            }],
        ),
    ])
