import utm
import time
import rclpy

import numpy as np
import terasim_cosim.redis_msgs as redis_msgs

from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from math import atan2, pi, cos, sin

from geometry_msgs.msg import PoseWithCovarianceStamped

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client


class GnssDecoder(Node):

    def __init__(self):
        super().__init__("gnss_decoder")

        self.sub_imu = self.create_subscription(Imu, "/ins/imu", self.imu_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/ins/odometry", self.odom_callback, 10
        )
        self.sub_nav_sat_fix = self.create_subscription(
            NavSatFix, "/ins/nav_sat_fix", self.gnss_callback, 10
        )

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/mcity/cav_pose", 10
        )

        self.timer = self.create_timer(0.02, self.on_timer)

        self.saved_odom_msg = None
        self.saved_imu_msg = None
        self.saved_nav_msg = None

        # Configure redis key-and data type
        key_value_config = {CAV_INFO: redis_msgs.ActorDict}
        self.redis_client = create_redis_client(key_value_config=key_value_config)

        self.UTM_offset = [-277497.10, -4686518.71]

        print("Reading GNSS info and set to cosim...")

    def on_timer(self):
        if self.saved_nav_msg and self.saved_odom_msg and self.saved_imu_msg:
            self.sync_gnss_cav_to_cosim()

    def sync_gnss_cav_to_cosim(self):
        if (
            self.saved_nav_msg is None
            or self.saved_odom_msg is None
            or self.saved_imu_msg is None
        ):
            return

        lat = self.saved_nav_msg.latitude
        lon = self.saved_nav_msg.longitude
        utm_coords = utm.from_latlon(lat, lon)

        x = utm_coords[0]
        y = utm_coords[1]
        z = self.saved_nav_msg.altitude

        orientation = self.get_vehicle_orientation()
        speed_long = self.get_vehicle_speed()

        # Modify position based on some offsets and set to pose message
        pose_with_cov_msg = PoseWithCovarianceStamped()
        pose_with_cov_msg.pose.pose.position.x = x + self.UTM_offset[0]
        pose_with_cov_msg.pose.pose.position.y = y + self.UTM_offset[1]

        # Extract and process orientation data
        qx, qy, qz, qw = self.get_quaternion_from_orientation(orientation)

        pose_with_cov_msg.pose.pose.orientation.x = qx
        pose_with_cov_msg.pose.pose.orientation.y = qy
        pose_with_cov_msg.pose.pose.orientation.z = qz
        pose_with_cov_msg.pose.pose.orientation.w = qw

        self.pub_pose.publish(pose_with_cov_msg)

        # For detailed fileds, see redis_msgs/VehicleDict.py
        cav_info = redis_msgs.ActorDict()

        # Set the timestamp
        cav_info.header.timestamp = time.time()

        # For detailed fileds, see redis_msgs/Vehicle.py
        cav = redis_msgs.Actor()
        cav.x = x
        cav.y = y
        cav.z = z
        cav.length = 5.0
        cav.width = 1.8
        cav.height = 1.5
        cav.orientation = orientation
        cav.speed_long = speed_long

        # Add bv to terasim_cosim_vehicle_info. You can add as many vehicles as you want.
        cav_info.data["CAV"] = cav

        self.redis_client.set(CAV_INFO, cav_info)

        self.saved_nav_msg = None
        self.saved_odom_msg = None
        self.saved_imu_msg = None

    def get_vehicle_speed(self):
        vx = self.saved_odom_msg.twist.twist.linear.x
        vy = self.saved_odom_msg.twist.twist.linear.y
        vz = self.saved_odom_msg.twist.twist.linear.z

        speed = np.sqrt(vx**2 + vy**2 + vz**2)

        # filter out noise
        if speed < 0.05:
            speed = 0.00

        return speed

    def get_vehicle_orientation(self):
        qx = self.saved_imu_msg.orientation.x
        qy = self.saved_imu_msg.orientation.y
        qz = self.saved_imu_msg.orientation.z
        qw = self.saved_imu_msg.orientation.w

        # create the quaternion
        quat = R.from_quat([qx, qy, qz, qw])

        # define the rotations
        #rotationAroundX = R.from_euler("x", 90, degrees=True)
        #rotationAroundY = R.from_euler("y", -90, degrees=True)
        #rotationAroundZ = R.from_euler("z", 90, degrees=True)

        # apply the rotations
        #quat = quat * rotationAroundX * rotationAroundY * rotationAroundZ
        #q_ned_to_enu = R.from_quat([np.sqrt(2)/2,np.sqrt(2)/2,0.0,0.0])
        #quat_enu = q_ned_to_enu * quat
        # extract the updated quaternion components
        #qx, qy, qz, qw = quat.as_quat()
        qx, qy, qz, qw = quat.as_quat()

        orientation = self.get_orientation_from_quaternion(qx, qy, qz, qw)

        return orientation

    def get_orientation_from_quaternion(self, qx, qy, qz, qw):
        orientation = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        while orientation > pi:
            orientation -= 2.0 * pi
        while orientation < -pi:
            orientation += 2.0 * pi

        return orientation

    def get_quaternion_from_orientation(self, orientation):
        w = cos(orientation / 2.0)
        x = 0.0
        y = 0.0
        z = sin(orientation / 2.0)

        return x, y, z, w

    def imu_callback(self, msg):
        self.saved_imu_msg = msg

    def odom_callback(self, msg):
        self.saved_odom_msg = msg

    def gnss_callback(self, msg):
        self.saved_nav_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = GnssDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
