# Copyright 2019 Autoware Foundation
# Licensed under the Apache License, Version 2.0

import rclpy, math
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from mcity_msgs.msg import VehicleState  # adjust this if your message package differs
from builtin_interfaces.msg import Time


STEER_TO_TIRE_RATIO = 17.0  # Adjust according to your vehicle configuration


class VehicleReportNode(Node):
    def __init__(self):
        super().__init__('vehicle_report_node')

        self.veh_state_msg = VehicleState()

        self.pub_vel_report = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)
        self.pub_steer_report = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)

        self.sub_veh_state = self.create_subscription(
            VehicleState,
            '/mcity/vehicle_state',
            self.veh_state_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_vehicle_report)  # 10 Hz

    def veh_state_callback(self, msg):
        self.veh_state_msg = msg

    def publish_vehicle_report(self):
        # Publish Velocity Report
        vel_msg = VelocityReport()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.longitudinal_velocity = self.veh_state_msg.speed_x

        # Publish Steering Report
        steer_msg = SteeringReport()
        steer_msg.stamp = self.get_clock().now().to_msg()
        steer_msg.steering_tire_angle = self.veh_state_msg.steer_state / STEER_TO_TIRE_RATIO

        self.pub_vel_report.publish(vel_msg)
        self.pub_steer_report.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleReportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
