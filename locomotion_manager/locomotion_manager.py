import rclpy
from rclpy.node import Node

from statemachine import StateMachine, State


class LocomotionModeMachine(StateMachine):

    def __init__(self):
        self.Modes = self.getModes()

    def getModes(self):
        modes = set()

        simple_rover_locomotion_mode = State(
            'SimpleRoverLocomotion', initial=True)
        crabbing_mode = State('Crabbing')

        modes.add(simple_rover_locomotion_mode)
        modes.add(crabbing_mode)

        return modes


class LocomotionManager(Node):
    def __init__(self):
        self.node_name = 'locomotion_manager_node'
        super().__init__(self.node_name)

        self.StateMachine = LocomotionModeMachine()

    def getModes(self):
        return self.StateMachine.Modes


def main(args=None):
    rclpy.init(args=args)

    locomotion_manager = LocomotionManager()

    modes = locomotion_manager.getModes()

    for mode in modes:
        locomotion_manager.get_logger().info(mode.name)

    rclpy.spin(locomotion_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
