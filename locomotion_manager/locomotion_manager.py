import rclpy
from rclpy.node import Node

from statemachine import StateMachine, State


class LocomotionModeMachine(StateMachine):

    def __init__(self):
        # self.Modes = self.getModes()
        self.States = set()
        self.Transitions = set()

    # def getModes(self):
        # modes = set()

        # simple_rover_locomotion_mode = State(
        #     'SimpleRoverLocomotion', initial=True)
        # crabbing_mode = State('Crabbing')

        # modes.add(simple_rover_locomotion_mode)
        # modes.add(crabbing_mode)

        # return modes

    def setModes(self, locomotion_modes):
        for mode in locomotion_modes:
            self.States.add(State(mode))

    '''
    Creates a state for each locomotion mode and the transitions between all states
    '''

    def setup(self, locomotion_modes):
        for mode in locomotion_modes:
            self.States.add(State(mode))

        for state_from in self.States:
            for state_to in self.States:
                if state_from != state_to:
                    self.Transitions.add(state_from.to(state_to))

        print(self.Transitions)


class LocomotionManager(Node):
    def __init__(self):
        self.node_name = 'locomotion_manager_node'
        super().__init__(self.node_name)

        self.StateMachine = LocomotionModeMachine()

        self.declare_parameter('locomotion_modes')
        # print(self.get_parameter('locomotion_modes')._value)

        # self.StateMachine.setModes(
        #     self.get_parameter('locomotion_modes')._value)
        self.StateMachine.setup(self.get_parameter('locomotion_modes')._value)

    def getStates(self):
        return self.StateMachine.States


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
