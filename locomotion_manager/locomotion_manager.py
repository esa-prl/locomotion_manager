import rclpy
from rclpy.node import Node
from rover_msgs.srv import ChangeLocomotionMode
from rover_msgs.msg import JointCommand

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
            state = self.create_state(mode)

    def get_states(self):
        """ Return all states """
        return self.states

    def create_state(self, name):
        """ Create a new state
        @param name: Name of the state to be created
        """

        enable_service_name = '{}/{}/enable'.format(self.namespace, name)
        enable_service = self.node.create_client(Trigger, enable_service_name, callback_group=self.node.cb_group)
        if not enable_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Could not find enable service for mode {}'.format(name))

        disable_service_name = '{}/{}/disable'.format(self.namespace, name)
        disable_service = self.node.create_client(Trigger, disable_service_name)
        if not enable_service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Could not find disable service for mode {}'.format(name))

        state = self.State(name, enable_service, disable_service)

        self.states.add(state)

        return state

    async def change_state(self, new_state):
        for state in self.states:
            if state.name == new_state:
                if self.active_state is None:
                    # Enable new mode
                    future = state.enable()

                    try:
                        result = await future
                    except Exception as e:
                        self.node.get_logger().info('Enable locomotion mode service call failed %r' % (e,))
                    else:
                        if result.success:
                            self.active_state = state
                            self.node.get_logger().info('Set {} as first active mode'.format(state.name))
                            return True
                        else:
                            self.node.get_logger().info('Could not set {}'.format(state.name))
                            return False
                elif state.name is not self.active_state.name:
                    # Disable active mode
                    self.node.get_logger().info('Start disabling')
                    future_disable = self.active_state.disable()
                    try:
                        result = await future_disable
                    except Exception as e:
                        self.node.get_logger().info('Disable locomotion mode service call failed %r' % (e,))
                    else:
                        if result.success:
                            self.active_state = None
                            self.node.get_logger().info('Set {} as first active mode'.format(state.name))
                        else:
                            self.node.get_logger().info('Could not set {}'.format(state.name))
                            return False

                    # Enable new mode
                    future_enable = state.enable()

                    try:
                        result = await future_enable
                    except Exception as e:
                        self.node.get_logger().info('Enable locomotion mode service call failed %r' % (e,))
                    else:
                        if result.success:
                            self.active_state = state
                            self.node.get_logger().info('Change from {} to {}'.format(self.active_state.name, state.name))
                            return True
                        else:
                            self.node.get_logger().info('Could not set {}'.format(state.name))
                            return False
                else:
                    self.node.get_logger().info('Requested state is already active')
                    return True

                break
            else:
                pass

        # If the new_state was not found in the available states
        self.node.get_logger().error('The requested locomotion mode is not defined')
        return False

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

        self.cb_group = ReentrantCallbackGroup()

        # Setup state machine
        self.state_machine = StateMachine(self)
        self.state_machine.setup(self.get_parameter('locomotion_modes')._value)

        # Setup service
        self.change_mode_server = self.create_service(
            ChangeLocomotionMode, 'change_locomotion_mode', self.change_locomotion_mode_service_callback)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def getStates(self):
        return self.state_machine.states

    async def change_locomotion_mode_service_callback(self, request, response):
        self.get_logger().debug('Locomotion mode change request: %s' %
                                request.new_locomotion_mode)

        future = self.state_machine.change_state(request.new_locomotion_mode)
        result = await future

        if result is True:
            response.success = True
        else:
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    locomotion_manager = LocomotionManager()

    states = locomotion_manager.getStates()

    rclpy.spin(locomotion_manager)
    locomotion_manager.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
