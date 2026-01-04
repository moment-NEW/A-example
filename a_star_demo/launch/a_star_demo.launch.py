from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="a_star_demo",
                executable="a_star_node",
                name="a_star_node",
                output="screen",
                parameters=[
                    {
                        "start_x": 0,
                        "start_y": 0,
                        "goal_x": 9,
                        "goal_y": 9,
                        "frame_id": "map",
                    }
                ],
            )
        ]
    )
