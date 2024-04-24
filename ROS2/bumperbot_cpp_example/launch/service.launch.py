from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    client_node = Node(
        package="bumperbot_cpp_example",
        executable="simple_service_client",
        arguments=["2","3"],
    )

    server_node = Node(
        package="bumperbot_cpp_example",
        executable="simple_service_server",
    )

    return LaunchDescription([
        server_node,
        client_node,
    ])