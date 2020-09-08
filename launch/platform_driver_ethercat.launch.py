import ament_index_python
import launch
import launch_ros
import lifecycle_msgs
import os

def generate_launch_description():
    default_pd_config_file_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('platform_driver_ethercat_ros2'),
        'config',
        'marta.yaml'
    )

    # Launch declarations
    declare_pd_config_file_path_cmd = launch.actions.DeclareLaunchArgument(
        'pd_config_file_path',
        default_value = default_pd_config_file_path,
        description = 'Full path to the platform_driver_ethercat_ros2 config file'
    )

    # Create pd node
    pd_node = launch_ros.actions.LifecycleNode(
        package = 'platform_driver_ethercat_ros2',
        executable = 'platform_driver_ethercat_node',
        name = 'platform_driver_ethercat_node',
        parameters = [{'config_file': launch.substitutions.LaunchConfiguration('pd_config_file_path')}]
    )

    # Make the pd node take the 'configure' transition
    pd_configure_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Make the pd node take the 'activate' transition
    pd_activate_event = launch.actions.EmitEvent(
        event = launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(pd_node),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        )
    )

    # When the pd node reaches the 'inactive' state from the 'unconfigured' state, make it take the 'activate' transition
    pd_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node = pd_node,
            start_state = 'configuring',
            goal_state = 'inactive',
            entities = [pd_activate_event]
        )
    )

    return launch.LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.
        #launch.actions.SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        declare_pd_config_file_path_cmd,
        pd_inactive_state_handler,
        pd_node,
        pd_configure_event,
    ])
