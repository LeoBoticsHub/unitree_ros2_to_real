import rclpy
from rclpy.node import Node

from datetime import datetime
from io import TextIOWrapper

from ros2_unitree_legged_real.unitreeRos2Client import UnitreeRos2Client

class TestRos2ControlNode(Node):
    """Ros 2 node for testing if the unitree ros 2 controller works"""

    file: TextIOWrapper

    def __init__(self):
        super().__init__("test_ros2_node")

        # getting current date for file name
        now = datetime.now()
        data_str = now.strftime("%d%m%Y_%H%M%S")

        # open file
        self.file = open("test_recap_" + data_str + ".txt", "w")

        # initialize unitree service clients and publishers
        self.client = UnitreeRos2Client(self)


    def __del__(self):
        self.file.close()


    def check_if_done(self, test_name) -> None:
        self.get_logger().info(f"executed test {test_name}.")
        resp = ""
        yes = ["y", "yes"]
        no = ["n", "no"]
        while not (resp in yes or resp in no):
            self.get_logger().info(f"Has the test been succesful? [y/n]")
            print(resp)
            resp = input().lower()
            if resp in yes:
                self.file.write(f"TEST:\n{test_name} -> SUCCESS\n\n")
            elif resp in no:
                self.file.write(f"TEST:\n{test_name} -> !ERROR!\n\n")
                self.get_logger().info(f"Would you like to add a comment? [y/n]")
                value = input().lower()
                if value in yes:
                    self.get_logger().info(f"Insert your comment:")
                    value = input().lower()
                    self.file.write("COMMENT:\n" + value + "\n")
            else:
                self.get_logger().warn("Please enter a y/yes or n/no answer.")
                

    def execute_test(self):

        ### STARTING TESTS ###
        self.get_logger().warn("Test is starting! Please press enter to continue.")
        input()

        # Stand Up Test
        self.client.standUp()
        self.check_if_done("stand_up_test")

        # Battery Test
        battery = self.client.getBatterySoc()
        self.get_logger().info(f"battery level: {battery}")
        self.check_if_done("battery_soc_test")

        # Body Height Test
        height = self.client.getBodyHeight()
        self.get_logger().info(f"Body height: {height} (it should be 0.54 more or less)")
        self.check_if_done("get_body_height_test")

        # Get Robot Mode Test
        mode = self.client.getRobotMode()
        if mode == 1:
            self.file.write(f"TEST: get_robot_mode_test -> SUCCESS\n\n")
        else:
            self.file.write(f"TEST: get_robot_mode_test -> !ERROR!\n\n")
        
        # Cmd vel tests
        self.client.pubCmdVel(0.3, 0.0, 0.0)
        self.check_if_done("move_forward_test")

        self.client.pubCmdVel(-0.3, 0.0, 0.0)
        self.check_if_done("move_backward_test")

        self.client.pubCmdVel(0.0, 0.3, 0.0)
        self.check_if_done("move_lateral_left_test")

        self.client.pubCmdVel(0.0, -0.3, 0.0)
        self.check_if_done("move_lateral_right_test")

        self.client.pubCmdVel(0.0, 0.0, 0.5)
        self.check_if_done("turn_counterclockwise_test")

        self.client.pubCmdVel(0.0, 0.0, -0.5)
        self.check_if_done("turn_clockwise_test")

        # Cmd orient tests

        self.client.pubCmdOrient(0.0, 0.15, 0.0)
        self.check_if_done("look_down_test")

        self.client.pubCmdOrient(0.0, -0.15, 0.0)
        self.check_if_done("look_up_test")

        self.client.pubCmdOrient(0.0, 0.0, 0.3)
        self.check_if_done("look_left_test")

        self.client.pubCmdOrient(0.0, 0.0, -0.3)
        self.check_if_done("look_right_test")

        # Foot Height test
        self.client.setFootHeight(0.1)
        self.client.pubCmdVel(0.3, 0.0, 0.0)
        self.check_if_done("footstep_height_test")
        self.client.setFootHeight(0.0)

        # Body Height test
        self.client.setBodyHeight(0.1)
        self.client.pubCmdVel(0.3, 0.0, 0.0)
        self.check_if_done("body_height_test")
        self.client.setBodyHeight(0.0)

        # Stand Down Test
        self.client.standDown()
        self.check_if_done("stand_down_test")

        # Power off Test
        self.client.powerOff()
        self.check_if_done("power_off_test")

        ### TESTS CONCLUDED ###
        self.get_logger().warn("Test finished!")


if __name__ == "__main__":

    rclpy.init()

    test_node = TestRos2ControlNode()

    test_node.execute_test()

    test_node.destroy_node()

    rclpy.shutdown()