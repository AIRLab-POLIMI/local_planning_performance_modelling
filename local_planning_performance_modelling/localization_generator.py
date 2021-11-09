# -*- coding: utf-8 -*-

import random
import numpy as np
import pyquaternion
import rclpy
import tf2_ros
import tf_transformations
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
import nav_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped, TransformStamped, Pose2D, Transform

from performance_modelling_py.utils import print_info


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

        # run variables
        self.pose_2d_gt = Pose2D()
        self.pose_2d_gen = Pose2D()

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

    def update_pose_timer_callback(self):
        if self.pose_2d_gt is None:
            return

        self.pose_2d_gen.x = random.normalvariate(self.pose_2d_gt.x, self.translation_error)
        self.pose_2d_gen.y = random.normalvariate(self.pose_2d_gt.y, self.translation_error)
        self.pose_2d_gen.theta = random.normalvariate(self.pose_2d_gt.theta, self.rotation_error)

        self.publish_pose()

    def ground_truth_pose_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        p = odometry_msg.pose.pose.position
        r = odometry_msg.pose.pose.orientation
        self.pose_2d_gt.x, self.pose_2d_gt.y = p.x, p.y
        self.pose_2d_gt.theta = pyquaternion.Quaternion(vector=[r.x, r.y, r.z], scalar=r.w).yaw_pitch_roll[0]

    def publish_tf_timer_callback(self):
        base_to_odom_tf = self.tf_buffer.lookup_transform(source_frame=self.robot_base_frame, target_frame=self.odom_frame, time=Time())
        print("base_to_odom_tf", base_to_odom_tf)

        map_to_base_tf = TransformStamped()
        map_to_base_tf.header.stamp = base_to_odom_tf.header.stamp
        map_to_base_tf.header.frame_id = self.fixed_frame
        map_to_base_tf.child_frame_id = self.robot_base_frame
        map_to_base_tf.transform.translation.x = self.pose_2d_gen.x
        map_to_base_tf.transform.translation.y = self.pose_2d_gen.y
        q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=self.pose_2d_gen.theta)
        map_to_base_tf.transform.rotation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        print("map_to_base_tf", map_to_base_tf)

        self.tf_buffer.set_transform(map_to_base_tf, "default_authority")

        map_to_odom_tf = self.tf_buffer.lookup_transform(source_frame=self.fixed_frame, target_frame=self.robot_base_frame, time=base_to_odom_tf.header.stamp)
        print("map_to_odom_tf", map_to_odom_tf)
        print()
        print()
        print()
        print()
        print()

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
