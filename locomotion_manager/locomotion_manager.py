import rclpy
from rclpy.node import Node
from rover_msgs.srv import ChangeLocomotionMode
from rover_msgs.msg import JointCommand

from std_srvs.srv import Trigger


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
        enable_service = self.node.create_client(Trigger, enable_service_name)

        disable_service_name = '{}/{}/disable'.format(self.namespace, name)
        disable_service = self.node.create_client(Trigger, disable_service_name)

        state = self.State(name, enable_service, disable_service)

        self.states.add(state)

        return state

    def change_state(self, new_state):
        """ Change to another state
        @param name: Name of the state to be created
        """
        for state in self.states:
            if state.name == new_state:
                if self.active_state is None:
                    # Enable new mode
                    state.enable()
                    self.active_state = state
                    self.node.get_logger().info('Set {} as first active mode'.format(state.name))
                elif state.name is not self.active_state.name:
                    # Disable active mode
                    self.active_state.disable()
                    # Enable new mode
                    self.node.get_logger().info('Change from {} to {}'.format(self.active_state.name, state.name))
                    state.enable()
                    self.active_state = state
                else:
                    self.node.get_logger().info('Requested state is already active')
                    # If the requested mode is already active do nothing

                break
        else:
            self.node.get_logger().error('The requested locomotion mode is not defined')
            return

    class State():
        def __init__(self, name, enable_service, disable_service):
            self.name = name

            self.enable_service = enable_service
            self.disable_service = disable_service

        def enable(self):
            request = Trigger.Request()
            self.enable_service.call_async(request)

        def disable(self):
            request = Trigger.Request()
            self.disable_service.call_async(request)


class LocomotionManager(Node):

    def __init__(self):
        # Init node
        self.node_name = 'locomotion_manager_node'
        super().__init__(self.node_name)

        # Declare parameters
        self.declare_parameter('locomotion_modes')

        # Setup state machine
        self.state_machine = StateMachine(self)
        self.state_machine.setup(self.get_parameter('locomotion_modes')._value)

        # Here I should create the services for the states

        # Setup service
        self.change_mode_server = self.create_service(
            ChangeLocomotionMode, 'change_locomotion_mode', self.change_locomotion_mode_service_callback)

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def getStates(self):
        return self.state_machine.states

    def change_locomotion_mode_service_callback(self, request, response):
        self.get_logger().debug('Locomotion mode change request: %s' %
                                request.new_locomotion_mode)

        # TODO: Check whether state exists return warning if not
        self.state_machine.change_state(request.new_locomotion_mode)

        response.success = True
        response.message = ''
        return response


def main(args=None):
    rclpy.init(args=args)

    locomotion_manager = LocomotionManager()

    states = locomotion_manager.getStates()

    for state in states:
        locomotion_manager.get_logger().info(state.name)

    rclpy.spin(locomotion_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
