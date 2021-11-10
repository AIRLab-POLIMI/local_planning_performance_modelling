# -*- coding: utf-8 -*-

import random
import numpy as np
import pyquaternion
import rclpy
import tf2_ros
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
import nav_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped, TransformStamped, Pose2D

from performance_modelling_py.utils import nanoseconds_to_seconds, print_error


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocalizationGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


class LocalizationGenerator(Node):
    def __init__(self):
        super().__init__('localization_generator_node', automatically_declare_parameters_from_overrides=True)

        # topics, services, actions, entities and frames names
        publish_tf_rate = self.get_parameter('publish_tf_rate').value
        update_pose_rate = self.get_parameter('update_pose_rate').value
        self.translation_error = self.get_parameter('translation_error').value
        self.rotation_error = self.get_parameter('rotation_error').value
        generated_pose_topic = self.get_parameter('generated_pose_topic').value
        ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.gt_timeout = self.get_parameter('ground_truth_timeout').value
        self.start_timeout = self.get_parameter('ground_truth_start_timeout').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value

        # run variables
        self.pose_2d_gt = Pose2D()
        self.pose_2d_gen = Pose2D()
        self.map_to_odom_pose_2d = Pose2D()
        self.start_time = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)
        self.last_gt_time = None

        # setup timers
        self.create_timer(1/publish_tf_rate, self.publish_tf_timer_callback)
        self.create_timer(1 / update_pose_rate, self.update_pose_timer_callback)

        # setup buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        # setup publishers
        self.generated_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, generated_pose_topic, qos_profile_sensor_data)

        # setup subscribers
        self.create_subscription(Odometry, ground_truth_pose_topic, self.ground_truth_pose_callback, qos_profile_sensor_data)

        self.get_logger().info(f"localization parameters:"
                               f"    translation_error: {self.translation_error}"
                               f"    rotation_error: {self.rotation_error}"
                               f"    update_pose_rate: {update_pose_rate}")

    def update_pose_timer_callback(self):
        now = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)
        if self.last_gt_time is None:
            if now - self.start_time > self.start_timeout:
                print_error(f"LocalizationGenerator: no ground truth message received (it has been {now - self.start_time} seconds)")
            return

        if now - self.last_gt_time > self.gt_timeout:
            print_error(f"LocalizationGenerator: last ground truth message older than {self.gt_timeout} seconds [{now - self.last_gt_time}]")

        self.pose_2d_gen.x = random.normalvariate(self.pose_2d_gt.x, self.translation_error)
        self.pose_2d_gen.y = random.normalvariate(self.pose_2d_gt.y, self.translation_error)
        self.pose_2d_gen.theta = random.normalvariate(self.pose_2d_gt.theta, self.rotation_error)

        def hc(p):
            return np.array([
                [np.cos(p.theta), -np.sin(p.theta), p.x],
                [np.sin(p.theta), np.cos(p.theta), p.y],
                [0, 0, 1],
            ])

        def pose_2d(m_hc):
            return Pose2D(x=m_hc[0, 2], y=m_hc[1, 2], theta=np.arctan2(m_hc[1, 0], m_hc[0, 0]))

        odom_to_base_tf = self.tf_buffer.lookup_transform(source_frame=self.robot_base_frame, target_frame=self.odom_frame, time=Time())
        r = odom_to_base_tf.transform.rotation
        odom_to_base_pose_2d = Pose2D(
            x=odom_to_base_tf.transform.translation.x,
            y=odom_to_base_tf.transform.translation.y,
            theta=pyquaternion.Quaternion(vector=[r.x, r.y, r.z], scalar=r.w).yaw_pitch_roll[0]
        )
        self.map_to_odom_pose_2d = pose_2d(hc(self.pose_2d_gen) @ np.linalg.inv(hc(odom_to_base_pose_2d)))

        self.publish_pose()

    def ground_truth_pose_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        self.last_gt_time = nanoseconds_to_seconds(Time.from_msg(odometry_msg.header.stamp).nanoseconds)
        p = odometry_msg.pose.pose.position
        r = odometry_msg.pose.pose.orientation
        self.pose_2d_gt.x, self.pose_2d_gt.y = p.x, p.y
        self.pose_2d_gt.theta = pyquaternion.Quaternion(vector=[r.x, r.y, r.z], scalar=r.w).yaw_pitch_roll[0]

    def publish_tf_timer_callback(self):
        map_to_odom_tf = TransformStamped()
        map_to_odom_tf.header.stamp = (self.get_clock().now() + Duration(seconds=self.transform_tolerance)).to_msg()
        map_to_odom_tf.header.frame_id = self.fixed_frame
        map_to_odom_tf.child_frame_id = self.odom_frame
        map_to_odom_tf.transform.translation.x = self.map_to_odom_pose_2d.x
        map_to_odom_tf.transform.translation.y = self.map_to_odom_pose_2d.y
        map_to_odom_q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=self.map_to_odom_pose_2d.theta)
        map_to_odom_tf.transform.rotation = Quaternion(w=map_to_odom_q.w, x=map_to_odom_q.x, y=map_to_odom_q.y, z=map_to_odom_q.z)
        self.br.sendTransform(map_to_odom_tf)

    def publish_pose(self):
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.fixed_frame
        p.pose.pose = Pose()
        p.pose.pose.position.x = self.pose_2d_gen.x
        p.pose.pose.position.y = self.pose_2d_gen.y
        q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=self.pose_2d_gen.theta)
        p.pose.pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        self.generated_pose_publisher.publish(p)
