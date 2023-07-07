""" A ROS 2 node to publish markers for visualization in RViz """
import os
import sys

# Append mission to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'mission'))

import argparse
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# For waypoints calculation
import yaml
from pydantic.error_wrappers import ValidationError
from std_msgs.msg import String

parser = argparse.ArgumentParser()
parser.add_argument('--calculator_params', type=str,
                    help='Scaled models')
parser.add_argument('--scaled', type=str, default='false',
                    help='Scaled models')
parser.add_argument('--use_sim_time', type=str, default='false',
                    help='Use sim time')

os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + os.environ['PWD'] + '/models'


class MarkerPublisherNode(Node):
    """ A ROS node to publish markers for visualization in RViz """

    def __init__(self, scale: float = 1.0, use_sim_time=False):
        """ Initialize the node """
        super().__init__('marker_publisher_node')

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.scale = 1.0
        self.marker_pub_0 = self.create_publisher(
            Marker, 'visualization_marker_1', 10)
        self.marker_pub_1 = self.create_publisher(
            Marker, 'visualization_marker_2', 10)
        self.marker_pub_2 = self.create_publisher(
            Marker, 'visualization_marker_3', 10)
        freq_uavs = 0.05
        freq_gates = 0.1
        self.timer_0 = self.create_timer(freq_uavs, self.publish_drone_0)
        self.timer_1 = self.create_timer(freq_uavs, self.publish_drone_1)
        self.timer_2 = self.create_timer(freq_gates, self.publish_gates)      

    def publish_drone_0(self):
        """ Publish the markers """
        quadrotor_marker_0 = self.create_quadrotor_marker('cf0/base_link', 0)
        
        self.marker_pub_0.publish(quadrotor_marker_0)
        
    def publish_drone_1(self):
        quadrotor_marker_1 = self.create_quadrotor_marker('cf1/base_link', 1)

        self.marker_pub_1.publish(quadrotor_marker_1)

    def publish_gates(self):
        gate_marker_0 = self.create_gate_marker('gate_0', 2)
        gate_marker_1 = self.create_gate_marker('gate_1', 3)

        self.marker_pub_2.publish(gate_marker_0)
        self.marker_pub_2.publish(gate_marker_1)

    def create_quadrotor_marker(self, frame_id: str, id: int):
        """ Create a quadrotor marker at TF frame drone0/base_link """
        quadrotor = Marker()
        quadrotor.header.stamp = self.get_clock().now().to_msg()
        quadrotor.header.frame_id = frame_id
        quadrotor.id = id
        quadrotor.ns = f'cf{id}'
        quadrotor.type = Marker.MESH_RESOURCE
        quadrotor.mesh_resource = 'package://as2_ign_gazebo_assets/models/quadrotor_base/meshes/quadrotor.dae'
        quadrotor.mesh_use_embedded_materials = True
        quadrotor.scale.x = self.scale 
        quadrotor.scale.y = self.scale 
        quadrotor.scale.z = self.scale 
        quadrotor.color.a = 1.0
        quadrotor.color.r = 1.0
        quadrotor.color.g = 1.0
        quadrotor.color.b = 1.0

        return quadrotor

    def create_gate_marker(self, frame_id: str, id: int):
        """ Create a gate marker """

        gate = Marker()
        gate.header.stamp = self.get_clock().now().to_msg()
        gate.header.frame_id = frame_id
        gate.id = id
        gate.ns = f'gate_{id}'
        gate.type = Marker.MESH_RESOURCE
        gate.mesh_resource = 'models/gate.dae'
        gate.mesh_use_embedded_materials = True
        gate.scale.x = self.scale 
        gate.scale.y = self.scale 
        gate.scale.z = self.scale
        gate.color.a = 1.0
        gate.color.r = 0.7
        gate.color.g = 0.7
        gate.color.b = 0.7

        return gate


def main(args=None):
    """ Main function """
    argument_parser = parser.parse_args()
    if argument_parser.scaled == 'true':
        print("Scaled mode")

    use_sim_time = False
    if argument_parser.use_sim_time == 'true':
        print("Use sim time")
        use_sim_time = True

    rclpy.init(args=args)
    marker_publisher_node = MarkerPublisherNode(
        use_sim_time=use_sim_time)
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
