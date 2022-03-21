from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():

    od_node = Node(
                    package='odrive_ros2',       
                    executable='odrive_node',
                    name='odrive_node',
                    output='screen'
                                )

    odrive_connection = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/connect_odrive ',
            'std_srvs/srv/Trigger ',
            
        ]],
        shell=True
    )
    axis0_state3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/request_state ',
            'odrive_interfaces/srv/AxisState ',
            ' "{axis: 0, state: 3}" '
        ]],
        shell=True
    )

    axis1_state3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/request_state ',
            'odrive_interfaces/srv/AxisState ',
            ' "{axis: 1, state: 3}" '
        ]],
        shell=True
    )

    axis0_state8 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/request_state ',
            'odrive_interfaces/srv/AxisState ',
            ' "{axis: 0, state: 8}" '
        ]],
        shell=True
    )

    axis1_state8 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            '/request_state ',
            'odrive_interfaces/srv/AxisState ',
            ' "{axis: 1, state: 8}" '
        ]],
        shell=True
    )
    
    return LaunchDescription([
        od_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action= od_node,
                on_start=[
                    LogInfo(msg='Looking for odrive device ... '),
                    odrive_connection,
                    TimerAction(
                        period= 10.0,
                        actions=[
                            LogInfo(msg=' Starting axis sequence '),
                            axis0_state3, axis1_state3],
                    
                    ),
                    TimerAction(
                        period= 30.0,
                        actions=[
                            LogInfo(msg=' Starting axis closed loop control '),
                            axis0_state8, axis1_state8],
                    )
                ]
            )
        ),
    ])