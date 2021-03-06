import rclpy
from rclpy.node import Node
from rover_msgs.srv import ChangeLocomotionMode

from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup


class StateMachine():
    def __init__(self, node):
        self.states = set()
        self.active_state = None
        self.node = node

        self.namespace = self.node.get_namespace()
        # Blank out namespace if there is None.
        # Otherwise the service names contain a double backslash //.
        if self.namespace == "/":
            self.namespace = ""

    def setup(self, locomotion_modes):
        """ Creates a state for each locomotion mode and the transitions between all states
        @param locomotion_modes: List of locomotion modes as strings
        """
        for mode in locomotion_modes:
            self.create_state(mode)

    def get_states(self):
        """ Return all states """
        return self.states

    def create_state(self, name):
        """ Create a new state
        @param name: Name of the state to be created
        """

        # Create enable service
        enable_service_name = '{}/{}/enable'.format(self.namespace, name)
        enable_service = self.node.create_client(Trigger,
                                                 enable_service_name,
                                                 callback_group=self.node.cb_group)

        # Wait for enable service to become available
        if not enable_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Could not find enable service for mode {}'.format(name))
            return

        # Create disable service
        disable_service_name = '{}/{}/disable'.format(self.namespace, name)
        disable_service = self.node.create_client(Trigger,
                                                  disable_service_name,
                                                  callback_group=self.node.cb_group)

        # Wait for service to become available
        if not disable_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Could not find disable service for mode {}'.format(name))
            return

        state = self.State(name, enable_service, disable_service)

        self.states.add(state)

        return state

    async def change_state(self, new_state):
        """ Change to a new state
        @param new_state: Name of the state to changed to
        """
        for state in self.states:
            if state.name == new_state:
                if self.active_state is None:
                    # Enable new mode
                    future = state.enable()
                    rclpy.spin_until_future_complete(self.node, future)

                    try:
                        result = future.result()
                    except Exception as e:
                        msg = 'Enable locomotion mode service call failed %r' % (e, )
                        self.node.get_logger().error(msg)
                        return False, msg
                    else:
                        if result.success:
                            self.active_state = state
                            msg = 'Set {} as first active mode'.format(state.name)
                            self.node.get_logger().info(msg)
                            return True, msg
                        else:
                            msg = 'Could not enable {}'.format(state.name)
                            self.node.get_logger().warn(msg)
                            return False, msg
                elif state.name is not self.active_state.name:
                    # Disable active mode
                    old_state_name = self.active_state.name
                    future_disable = self.active_state.disable()
                    rclpy.spin_until_future_complete(self.node, future_disable)

                    try:
                        result = future_disable.result()
                    except Exception as e:
                        msg = 'Disable locomotion mode service call failed %r' % (e, )
                        self.node.get_logger().error(msg)
                        return False, msg
                    else:
                        if result.success:
                            self.node.get_logger().info('Disable {}'.format(
                                self.active_state.name))
                            self.active_state = None
                        else:
                            msg = "Could not disable {}".format(self.active_state.name)
                            self.node.get_logger().info(msg)
                            return False, msg

                    # Enable new mode
                    future_enable = state.enable()
                    rclpy.spin_until_future_complete(self.node, future_enable)

                    try:
                        result = future_enable.result()
                    except Exception as e:
                        msg = 'Enable locomotion mode service call failed %r' % (e, )
                        self.node.get_logger().error(msg)
                        return False, msg
                    else:
                        if result.success:
                            self.active_state = state
                            msg = 'Change from {} to {}'.format(old_state_name, state.name)
                            self.node.get_logger().info(msg)
                            return True, msg
                        else:
                            msg = 'Could not enable {}'.format(state.name)
                            self.node.get_logger().warn(msg)
                            return False, msg
                else:
                    msg = '{} is already active'.format(state.name)
                    self.node.get_logger().info(msg)
                    return True, msg

                break
            else:
                pass

        # If the new_state was not found in the available states
        msg = 'The requested locomotion mode: {} is not defined'.format(new_state)
        self.node.get_logger().error(msg)
        return False, msg

    class State():
        def __init__(self, name, enable_service, disable_service):
            self.name = name

            self.enable_service = enable_service
            self.disable_service = disable_service

        def enable(self):
            request = Trigger.Request()
            return self.enable_service.call_async(request)

        def disable(self):
            request = Trigger.Request()
            return self.disable_service.call_async(request)


class LocomotionManager(Node):
    def __init__(self):
        # Init node
        self.node_name = 'locomotion_manager_node'
        super().__init__(self.node_name)

        # Declare parameters
        self.declare_parameter('locomotion_modes')

        # Callback group that allows service callbacks
        # to be executed in parallel
        self.cb_group = ReentrantCallbackGroup()

        # Setup state machine
        self.state_machine = StateMachine(self)
        self.state_machine.setup(self.get_parameter('locomotion_modes')._value)

        # Setup service to receive change_locomotion_mode requests
        self.change_mode_server = self.create_service(ChangeLocomotionMode,
                                                      'change_locomotion_mode',
                                                      self.change_locomotion_mode_service_callback)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def getStates(self):
        return self.state_machine.get_states()

    async def change_locomotion_mode_service_callback(self, request, response):
        self.get_logger().debug('Locomotion mode change request: %s' % request.new_locomotion_mode)

        future = self.state_machine.change_state(request.new_locomotion_mode)
        result, msg = await future

        response.success = result
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)

    locomotion_manager = LocomotionManager()

    # Has to use this instead of rclpy.spin(node)
    # Otherwise a second service call gets stuck
    try:
        while rclpy.ok():
            rclpy.spin_once(locomotion_manager)
    except KeyboardInterrupt:
        pass
    finally:
        locomotion_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
