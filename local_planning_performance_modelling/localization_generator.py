# -*- coding: utf-8 -*-

import random
from copy import copy

import numpy as np
import pyquaternion
import rclpy
import tf2_ros
import visualization_msgs.msg
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
import nav_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped, TransformStamped, Pose2D, PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker


from performance_modelling_py.utils import nanoseconds_to_seconds, print_error, print_info


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocalizationGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def ht(p):
    return np.array([
        [np.cos(p.theta), -np.sin(p.theta), p.x],
        [np.sin(p.theta), np.cos(p.theta), p.y],
        [0, 0, 1],
    ])


def pose_2d(m_ht):
    return Pose2D(x=m_ht[0, 2], y=m_ht[1, 2], theta=np.arctan2(m_ht[1, 0], m_ht[0, 0]))


def normalize_angle_difference(a_angle, b_angle):
    return np.abs(np.arctan2(np.sin(b_angle - a_angle), np.cos(b_angle - a_angle)))


def weighted_angle_average(a_angle, a_w, b_angle, b_w):
    if a_w + b_w == 0.:
        raise ZeroDivisionError
    return np.arctan2(a_w * np.sin(a_angle) + b_w * np.sin(b_angle), a_w * np.cos(a_angle) + b_w * np.cos(b_angle))


def get_closest_intersections(x0, y0, r0, x1, y1, r1):  # modified from https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles/55817881
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    # concentric
    if d == 0:
        return (x0, y0, r0 / 2 + r1 / 2), None

    # non intersecting
    if d > r0 + r1:
        a = r0 / 2 + d / 2 - r1 / 2
        x2 = x0 + (x1 - x0) / d * a
        y2 = y0 + (y1 - y0) / d * a
        return None, (x2, y2, x2, y2)

    # One circle within other
    if d < abs(r0 - r1):
        if r1 > r0:
            x0, y0, r0, x1, y1, r1 = x1, y1, r1, x0, y0, r0
        a = d / 2 + r1 / 2 + r0 / 2
        x2 = x0 + (x1 - x0) / d * a
        y2 = y0 + (y1 - y0) / d * a
        return None, (x2, y2, x2, y2)

    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = np.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d

        return None, (x3, y3, x4, y4)


class LocalizationGenerator(Node):
    def __init__(self):
        super().__init__('localization_generator_node', automatically_declare_parameters_from_overrides=True)

        # topics, services, actions, entities and frames names
        publish_tf_rate = self.get_parameter('publish_tf_rate').value
        update_pose_rate = self.get_parameter('update_pose_rate').value
        self.absolute_translation_error = self.get_parameter('translation_error').value
        self.absolute_rotation_error = self.get_parameter('rotation_error').value
        self.normalized_relative_translation_error = self.get_parameter('normalized_relative_translation_error').value
        self.normalized_relative_rotation_error = self.get_parameter('normalized_relative_rotation_error').value
        generated_pose_topic = self.get_parameter('generated_pose_topic').value
        ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.gt_timeout = self.get_parameter('ground_truth_timeout').value
        self.start_timeout = self.get_parameter('ground_truth_start_timeout').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value

        # run variables
        self.prev_pose_2d_gt = Pose2D()
        self.pose_2d_gt = Pose2D()
        self.pose_2d_gen = Pose2D()
        self.map_to_odom_pose_2d = Pose2D()
        self.last_update_pose_2d_gt = Pose2D()
        self.prev_pose_2d_rel = Pose2D()
        self.gt_poses_viz = list()
        self.rel_poses_viz = list()
        self.gen_poses_viz = list()
        self.start_time = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)
        self.last_gt_time = None
        self.trajectory_translation_sum = 0.
        self.trajectory_rotation_sum = 0.
        self.viz_circles_id = 0

        # setup timers
        self.create_timer(1/publish_tf_rate, self.publish_tf_timer_callback)
        self.create_timer(1 / update_pose_rate, self.update_pose_timer_callback)

        # setup buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        # setup publishers
        self.generated_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, generated_pose_topic, qos_profile_sensor_data)
        self.gt_poses_publisher = self.create_publisher(PoseArray, "/gt_poses", qos_profile_sensor_data)
        self.rel_poses_publisher = self.create_publisher(PoseArray, "/rel_poses", qos_profile_sensor_data)
        self.gen_poses_publisher = self.create_publisher(PoseArray, "/gen_poses", qos_profile_sensor_data)
        self.viz_marker_publisher = self.create_publisher(Marker, "/viz_abs", qos_profile_sensor_data)

        # setup subscribers
        self.create_subscription(Odometry, ground_truth_pose_topic, self.ground_truth_pose_callback, qos_profile_sensor_data)

        self.get_logger().info(f"localization parameters:\n"
                               f"    update_pose_rate: {update_pose_rate}\n"
                               f"    absolute_translation_error: {self.absolute_translation_error}\n"
                               f"    absolute_rotation_error: {self.absolute_rotation_error}\n"
                               f"    normalized_relative_translation_error: {self.normalized_relative_translation_error}\n"
                               f"    normalized_relative_rotation_error: {self.normalized_relative_rotation_error}"
                               )

    def update_pose_timer_callback(self):
        now = nanoseconds_to_seconds(self.get_clock().now().nanoseconds)
        if self.last_gt_time is None:
            if now - self.start_time > self.start_timeout:
                print_error(f"LocalizationGenerator: no ground truth message received (it has been {now - self.start_time} seconds)")
            return

        if now - self.last_gt_time > self.gt_timeout:
            print_error(f"LocalizationGenerator: last ground truth message older than {self.gt_timeout} seconds [{now - self.last_gt_time}]")

        print_info("", logger=self.get_logger().info)

        prev_pose_2d_gen = copy(self.pose_2d_gen)
        curr_pose_2d_gt = copy(self.pose_2d_gt)
        rel_gt_ht = np.linalg.inv(ht(self.last_update_pose_2d_gt)) @ ht(curr_pose_2d_gt)

        print_info("self.trajectory_rotation_sum ", self.trajectory_rotation_sum, logger=self.get_logger().info)
        relative_translation_error = self.normalized_relative_translation_error * self.trajectory_translation_sum
        relative_rotation_error = self.normalized_relative_rotation_error * self.trajectory_rotation_sum
        self.trajectory_translation_sum = 0.
        self.trajectory_rotation_sum = 0.

        rel_error_sample_pose_2d = Pose2D(
            x=random.normalvariate(0., relative_translation_error),
            y=random.normalvariate(0., relative_translation_error),
            theta=random.normalvariate(0., relative_rotation_error)
        )
        rel_ht_with_error = rel_gt_ht @ ht(rel_error_sample_pose_2d)
        curr_pose_2d_rel_with_error = pose_2d(ht(self.prev_pose_2d_rel) @ rel_ht_with_error)
        # self.publish_rel_poses(curr_pose_2d_rel_with_error)
        self.prev_pose_2d_rel = copy(curr_pose_2d_rel_with_error)

        pose_2d_abs = Pose2D()
        pose_2d_abs.x = random.normalvariate(curr_pose_2d_gt.x, self.absolute_translation_error)
        pose_2d_abs.y = random.normalvariate(curr_pose_2d_gt.y, self.absolute_translation_error)
        pose_2d_abs.theta = random.normalvariate(curr_pose_2d_gt.theta, self.absolute_rotation_error)

        # product of two normal pdf
        #   s_t = np.sqrt((s_a**2 * s_r**2) / (s_a**2 + s_r**2))
        #   x_t = (x_r * s_a**2 + x_a * s_r**2) / (s_a**2 + s_r**2)
        curr_pose_2d_rel = pose_2d(ht(prev_pose_2d_gen) @ rel_gt_ht)
        # s_tra = np.sqrt((self.absolute_translation_error ** 2 * relative_translation_error ** 2) / (self.absolute_translation_error ** 2 + relative_translation_error ** 2))
        # x_tra = (curr_pose_2d_rel.x * self.absolute_translation_error ** 2 + curr_pose_2d_gt.x * relative_translation_error ** 2) / (self.absolute_translation_error ** 2 + relative_translation_error ** 2)
        # y_tra = (curr_pose_2d_rel.y * self.absolute_translation_error ** 2 + curr_pose_2d_gt.y * relative_translation_error ** 2) / (self.absolute_translation_error ** 2 + relative_translation_error ** 2)
        s_rot = np.sqrt((self.absolute_rotation_error ** 2 * relative_rotation_error ** 2) / (self.absolute_rotation_error ** 2 + relative_rotation_error ** 2))
        x_rot = weighted_angle_average(curr_pose_2d_rel.theta, self.absolute_rotation_error ** 2 / (self.absolute_rotation_error ** 2 + relative_rotation_error ** 2),
                                       curr_pose_2d_gt.theta, relative_rotation_error ** 2 / (self.absolute_rotation_error ** 2 + relative_rotation_error ** 2)
                                       )

        self.publish_viz_circle(curr_pose_2d_rel, relative_translation_error, "rel")
        self.publish_rel_poses(curr_pose_2d_rel)
        self.publish_viz_circle(curr_pose_2d_gt, self.absolute_translation_error, "abs")
        self.publish_gt_poses(curr_pose_2d_gt)

        self.pose_2d_gen = Pose2D()
        self.pose_2d_gen.theta = random.normalvariate(x_rot, s_rot)

        intersection_circle, intersection_points = get_closest_intersections(
            curr_pose_2d_rel.x, curr_pose_2d_rel.y, relative_translation_error,
            curr_pose_2d_gt.x, curr_pose_2d_gt.y, self.absolute_translation_error
        )
        if intersection_points is not None:
            print_info("intersection_points:", intersection_points, logger=self.get_logger().info)
            i_x1, i_y1, i_x2, i_y2 = intersection_points
            self.pose_2d_gen.x, self.pose_2d_gen.y = random.choice([(i_x1, i_y1), (i_x2, i_y2)])
        elif intersection_circle is not None:
            print_info("intersection_circle:", intersection_circle, logger=self.get_logger().info)
            i_x, i_y, i_r = intersection_circle
            rnd_angle = random.uniform(0, 2*np.pi)
            self.pose_2d_gen.x, self.pose_2d_gen.y = i_x + i_r*np.cos(rnd_angle), i_y + i_r*np.sin(rnd_angle)
        else:
            print_error("no points nor circle from get_closest_intersections:", logger=self.get_logger().error)

        # self.pose_2d_gen.x = random.normalvariate(x_tra, s_tra)
        # self.pose_2d_gen.y = random.normalvariate(y_tra, s_tra)
        print_info("self.pose_2d_gen:", self.pose_2d_gen, logger=self.get_logger().info)
        self.publish_gen_poses(self.pose_2d_gen)

        # print_info("curr_pose_2d_rel.theta\n\t", curr_pose_2d_rel.theta, logger=self.get_logger().info)
        # print_info("curr_pose_2d_gt.theta\n\t", curr_pose_2d_gt.theta, logger=self.get_logger().info)
        # print_info("x_rot\n\t", x_rot, logger=self.get_logger().info)

        odom_to_base_tf = self.tf_buffer.lookup_transform(source_frame=self.robot_base_frame, target_frame=self.odom_frame, time=Time())
        r = odom_to_base_tf.transform.rotation
        odom_to_base_pose_2d = Pose2D(
            x=odom_to_base_tf.transform.translation.x,
            y=odom_to_base_tf.transform.translation.y,
            theta=pyquaternion.Quaternion(vector=[r.x, r.y, r.z], scalar=r.w).yaw_pitch_roll[0]
        )
        self.map_to_odom_pose_2d = pose_2d(ht(self.pose_2d_gen) @ np.linalg.inv(ht(odom_to_base_pose_2d)))  # map_to_odom = map->base ⊕ base->odom

        self.publish_pose()
        self.last_update_pose_2d_gt = copy(curr_pose_2d_gt)

    def ground_truth_pose_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        self.last_gt_time = nanoseconds_to_seconds(Time.from_msg(odometry_msg.header.stamp).nanoseconds)
        p = odometry_msg.pose.pose.position
        r = odometry_msg.pose.pose.orientation
        self.pose_2d_gt.x, self.pose_2d_gt.y = p.x, p.y
        self.pose_2d_gt.theta = pyquaternion.Quaternion(vector=[r.x, r.y, r.z], scalar=r.w).yaw_pitch_roll[0]

        # compute the integral of translation and rotation, for normalization of errors between updates
        self.trajectory_translation_sum += np.sqrt((self.prev_pose_2d_gt.x - self.pose_2d_gt.x)**2 + (self.prev_pose_2d_gt.y - self.pose_2d_gt.y)**2)
        self.trajectory_rotation_sum += normalize_angle_difference(self.prev_pose_2d_gt.theta, self.pose_2d_gt.theta)

        self.prev_pose_2d_gt = copy(self.pose_2d_gt)

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

    def publish_viz_circle(self, pose_2d_msg, r, ns):
        pose_msg = Pose()
        pose_msg.position.x = pose_2d_msg.x
        pose_msg.position.y = pose_2d_msg.y
        pose_q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=pose_2d_msg.theta)
        pose_msg.orientation = Quaternion(w=pose_q.w, x=pose_q.x, y=pose_q.y, z=pose_q.z)

        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = self.viz_circles_id
        marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.points = [Point(x=r*np.cos(t), y=r*np.sin(t)) for t in np.linspace(0, 2*np.pi, 50)]
        marker.pose = pose_msg
        marker.scale.x = 0.001
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.viz_marker_publisher.publish(marker)
        self.viz_circles_id += 1

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

    def publish_gt_poses(self, gt_pose_2d_msg):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.fixed_frame
        gt_pose_msg = Pose()
        gt_pose_msg.position.x = gt_pose_2d_msg.x
        gt_pose_msg.position.y = gt_pose_2d_msg.y
        gt_pose_q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=gt_pose_2d_msg.theta)
        gt_pose_msg.orientation = Quaternion(w=gt_pose_q.w, x=gt_pose_q.x, y=gt_pose_q.y, z=gt_pose_q.z)
        self.gt_poses_viz.append(gt_pose_msg)
        pose_array_msg.poses = self.gt_poses_viz
        self.gt_poses_publisher.publish(pose_array_msg)

    def publish_rel_poses(self, rel_pose_2d_msg):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.fixed_frame
        rel_pose_msg = Pose()
        rel_pose_msg.position.x = rel_pose_2d_msg.x
        rel_pose_msg.position.y = rel_pose_2d_msg.y
        rel_pose_q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=rel_pose_2d_msg.theta)
        rel_pose_msg.orientation = Quaternion(w=rel_pose_q.w, x=rel_pose_q.x, y=rel_pose_q.y, z=rel_pose_q.z)
        self.rel_poses_viz.append(rel_pose_msg)
        pose_array_msg.poses = self.rel_poses_viz
        self.rel_poses_publisher.publish(pose_array_msg)

    def publish_gen_poses(self, gen_pose_2d_msg):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.fixed_frame
        gen_pose_msg = Pose()
        gen_pose_msg.position.x = gen_pose_2d_msg.x
        gen_pose_msg.position.y = gen_pose_2d_msg.y
        gen_pose_q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=gen_pose_2d_msg.theta)
        gen_pose_msg.orientation = Quaternion(w=gen_pose_q.w, x=gen_pose_q.x, y=gen_pose_q.y, z=gen_pose_q.z)
        self.gen_poses_viz.append(gen_pose_msg)
        pose_array_msg.poses = self.gen_poses_viz
        self.gen_poses_publisher.publish(pose_array_msg)
