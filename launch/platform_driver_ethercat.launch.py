import launch
import launch_ros
import lifecycle_msgs

def generate_launch_description():
    ld = launch.LaunchDescription()

    # Create pd node
    pd_node = launch_ros.actions.LifecycleNode(
            package = 'platform_driver_ethercat_ros2',
            node_executable = 'platform_driver_ethercat_node',
            node_name = 'platform_driver_ethercat_node',
            output = 'screen',
            arguments = ['--ros-args', '--log-level', 'debug'],
            parameters = [{'config_file': '/home/marta/ros/dev_ws/src/platform_driver_ethercat_ros2/config/marta.yaml'}]
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
            start_state = 'unconfigured',
            goal_state = 'inactive',
            entities = [pd_activate_event]
        )
    )

    ld.add_action(pd_inactive_state_handler)
    ld.add_action(pd_node)
    ld.add_action(pd_configure_event)

    return ld
