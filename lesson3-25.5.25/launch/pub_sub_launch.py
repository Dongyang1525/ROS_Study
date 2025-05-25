from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import GroupAction, DeclareLaunchArgument
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    #下面的代码是一个ROS2的launch文件，用于启动一个包含多个节点的ROS2应用程序
    #定义两个量来获取相关的参数
    # Create the launch configuration variables
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")

    # 每行日志在输出之前会被缓冲
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # 日志输出将使用颜色
    colorized_output_envvar = SetEnvironmentVariable(
        "RCUTILS_COLORIZED_OUTPUT", "1"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",#启动参数
        default_value="True",
        description="Whether to use composed bringup",
    )


    #声明一个组件容器的命令
    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="ros_learning_container",
        description="the name of conatiner that nodes will load in if use composition",
    )
    # 启动一个隔离的组件容器，用于运行组件化的 ROS 2 节点
    # 组合节点容器是一个特殊的节点，它可以包含其他节点
    # 组合节点容器的作用是将多个节点组合在一起，形成一个新的节点
    # 组合节点容器的名称可以通过参数传递

    start_component_container_isolated = Node(
        condition=IfCondition(use_composition),
        name=container_name,
        package="rclcpp_components",
        executable="component_container_isolated",
        arguments=["--ros-args", "--log-level", "info"],
        output="screen",
    )

    #组合节点和非组合节点的加载
    

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            Node(
                package="pub_sub_cmd_vel",
                executable="velocity_publisher_node",
                name="velocity_publisher",
                output="screen",
                parameters=[],
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="pub_sub_cmd_vel",
                executable="velocity_subscriber_node",
                name="velocity_subscriber",
                output="screen",
                parameters=[],
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="pub_sub_cmd_vel",
                plugin="velocity_publisher::VelocityPublisher",
                name="velocity_publisher",
                parameters=[],
            ),
            ComposableNode(
                package="pub_sub_cmd_vel",
                plugin="velocity_subscriber::VelocitySubscriber",
                name="velocity_subscriber",
                parameters=[],
            ),
        ],
    )

    # ld的创建和填充
    # ld的作用是将所有的节点和动作组合在一起

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_component_container_isolated)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
