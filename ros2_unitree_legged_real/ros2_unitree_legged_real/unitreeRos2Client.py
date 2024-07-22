import rclpy
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Vector3, Twist
from rclpy.node import Node

from ros2_unitree_legged_msgs.srv import GetBatteryState, GetFloat, GetInt, SetBodyOrientation, SetFloat

class UnitreeRos2Client:
    """ Unitree ros services and subscriber service interface

    It can be used to simplify interface with unitree services spawned by unitree_high_ros2_control node
    """

    def __init__(self, node: Node) -> None:

        self.node = node

        self.power_off_srv = self.node.create_client(Trigger, "power_off")
        self.emergency_button_srv = self.node.create_client(Trigger, "emergency_stop")
        self.stand_up_srv = self.node.create_client(Trigger, "stand_up")
        self.stand_down_srv = self.node.create_client(Trigger, "stand_down")
        self.get_mode_srv = self.node.create_client(GetInt, "get_robot_mode")
        self.battery_state_srv = self.node.create_client(GetBatteryState, "get_battery_state")
        self.set_foot_height_srv = self.node.create_client(SetFloat, "set_foot_height")
        self.set_body_height_srv = self.node.create_client(SetFloat, "set_body_height")
        self.get_foot_height_srv = self.node.create_client(GetFloat, "get_foot_height")
        self.get_body_height_srv = self.node.create_client(GetFloat, "get_body_height")

        self.cmd_vel_pub = self.node.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_orient_pub = self.node.create_publisher(Vector3, "cmd_orient", 1)


    def powerOff(self) -> bool:
        """The power off service interface

        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = Trigger.Request()
        future = self.power_off_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: Trigger.Response
        res = future.result()

        return res.success


    def emergencyStop(self) -> bool:
        """The emergency stop service interface

        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = Trigger.Request()
        future = self.emergency_button_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: Trigger.Response
        res = future.result()
        if not res.success:
            self.node.get_logger().error(res.message)

        return res.success


    def standUp(self) -> bool:
        """The stand up service interface

        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = Trigger.Request()
        future = self.stand_up_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: Trigger.Response
        res = future.result()
        if not res.success:
            self.node.get_logger().error(res.message)

        return res.success


    def standDown(self) -> bool:
        """The stand down service interface

        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = Trigger.Request()
        future = self.stand_down_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: Trigger.Response
        res = future.result()
        if not res.success:
            self.node.get_logger().error(res.message)

        return res.success


    def getRobotMode(self) -> int:
        """The get robot mode service interface

        Returns:
            int - The current robot state
        """
        req = GetInt.Request()
        future = self.get_mode_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: GetInt.Response
        res = future.result()
        
        self.node.get_logger().info(res.message)

        return res.value


    def getBatterySoc(self) -> float:
        """The get battery state-of-charge service interface

        Returns:
            float - The current robot battery state-of-charge in percentage
        """
        req = GetBatteryState.Request()
        future = self.battery_state_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: GetBatteryState.Response
        res = future.result()
        
        return res.battery_state.percentage


    def setFootHeight(self, height: float) -> bool:
        """The set foot height service interface

        Args:
            height: [float] the height delta value to add to the current foot height
        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = SetFloat.Request()
        req.value = height
        future = self.set_foot_height_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: SetFloat.Response
        res = future.result()

        if res.success:
            self.node.get_logger().info(res.message)
        else:
            self.node.get_logger().error(res.message)
            
        return res.success


    def getFootHeight(self) -> float:
        """The get foot height service interface

        Returns:
            float - The current robot foot height
        """
        req = GetFloat.Request()
        future = self.get_foot_height_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: GetFloat.Response
        res = future.result()

        return res.value


    def setBodyHeight(self, height: float) -> bool:
        """The set body height service interface

        Args:
            height: [float] the height delta value to add to the current body height
        Returns:
            bool - True if the service has been correctly executed, False otherwise
        """
        req = SetFloat.Request()
        req.value = height
        future = self.set_body_height_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: SetFloat.Response
        res = future.result()

        if res.success:
            self.node.get_logger().info(res.message)
        else:
            self.node.get_logger().error(res.message)
            
        return res.success


    def getBodyHeight(self) -> float:
        """The get body height service interface

        Returns:
            float - The current robot body height
        """
        req = GetFloat.Request()
        future = self.get_body_height_srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        res: GetFloat.Response
        res = future.result()

        return res.value


    def pubCmdVel(self, vx: float, vy: float, w: float) -> None:
        """The cmd_vel publisher interface

        Args:
            vx: [float] the frontal linear velocity
            vy: [float] the lateral linear velocity
            w: [float] the angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.angular.z = w

        self.cmd_vel_pub.publish(cmd_vel)


    def pubCmdOrient(self, roll: float, pitch: float, yaw: float) -> None:
        """The cmd_orient publisher interface

        Args:
            roll: [float] the roll delta angle angle
            pitch: [float] the pitch delta angle angle
            yaw: [float] the yaw delta angle angle
        """
        cmd_orient = Vector3()
        cmd_orient.x = roll
        cmd_orient.y = pitch
        cmd_orient.z = yaw

        self.cmd_orient_pub.publish(cmd_orient)