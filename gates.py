import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class Gates(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('gates')

        self.pose_0_sub = self.create_subscription(
            PoseStamped, '/gate_0/ground_truth/pose', self.gate_0_callback, qos_profile_sensor_data)

        self.pose_1_sub = self.create_subscription(
            PoseStamped, '/gate_1/ground_truth/pose', self.gate_1_callback, qos_profile_sensor_data)

        self.gate_0_pose = PoseStamped()
        self.gate_1_pose = PoseStamped()
        self.wait_pose_0 = False
        self.wait_pose_1 = False
        rclpy.spin_once(self)

    def gate_0_callback(self, pose_msg: PoseStamped):
        self.gate_0_pose = pose_msg
        self.wait_pose_0 = True

    def gate_1_callback(self, pose_msg: PoseStamped):
        self.gate_1_pose = pose_msg
        self.wait_pose_1 = True

    def get_gate_0_pose(self):
        while (not self.wait_pose_0):
            pass
        ret = [self.gate_0_pose.pose.position.x, self.gate_0_pose.pose.position.y,
               self.gate_0_pose.pose.position.z, self.gate_0_pose.pose.orientation.x,
               self.gate_0_pose.pose.orientation.y, self.gate_0_pose.pose.orientation.z,
               self.gate_0_pose.pose.orientation.w]

        return ret

    def get_gate_1_pose(self):
        while (not self.wait_pose_1):
            pass
        ret = [self.gate_1_pose.pose.position.x, self.gate_1_pose.pose.position.y,
               self.gate_1_pose.pose.position.z, self.gate_1_pose.pose.orientation.x,
               self.gate_1_pose.pose.orientation.y, self.gate_1_pose.pose.orientation.z,
               self.gate_1_pose.pose.orientation.w]
        return ret

    def shutdown(self):
        self.destroy_node()
