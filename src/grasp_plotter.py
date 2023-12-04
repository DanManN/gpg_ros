import numpy as np
import pygpg
import transformations as tf

import rospy
from visualization_msgs.msg import Marker, MarkerArray


class GraspPlotter:

    def __init__(
        self,
        topic='/plot_grasps',
        finger_width=0.0065,
        outer_diameter=0.098,
        hand_depth=0.0613,
        hand_height=0.035,
    ):
        self.rviz_pub = rospy.Publisher(topic, MarkerArray, queue_size=1)
        self.finger_width = finger_width
        self.outer_diameter = outer_diameter
        self.hand_depth = hand_depth
        self.hand_height = hand_height

    def draw_grasps(self, grasps, frame='world'):
        markers = self.convert_to_visual_grasp_msg(grasps, frame)
        self.rviz_pub.publish(markers)

    def convert_to_visual_grasp_msg(self, grasps, frame_id):
        hw = 0.5 * self.outer_diameter - 0.5 * self.finger_width

        marker_array = MarkerArray()
        for i, hand in enumerate(grasps):
            frame = np.eye(4)
            frame[:3, 0] = hand.get_grasp_approach()
            frame[:3, 1] = hand.get_grasp_binormal()
            frame[:3, 2] = hand.get_grasp_axis()
            left_bottom = hand.get_grasp_bottom(
            ) - hw * hand.get_grasp_binormal()
            right_bottom = hand.get_grasp_bottom(
            ) + hw * hand.get_grasp_binormal()
            left_top = left_bottom + self.hand_depth * hand.get_grasp_approach()
            right_top = right_bottom + self.hand_depth * hand.get_grasp_approach()
            left_center = left_bottom + 0.5 * (left_top - left_bottom)
            right_center = right_bottom + 0.5 * (right_top - right_bottom)
            base_center = left_bottom + 0.5 * (right_bottom - left_bottom)
            base_center -= 0.01 * hand.get_grasp_approach()
            approach_center = base_center - 0.04 * hand.get_grasp_approach()

            finger_lwh = np.array(
                [
                    self.hand_depth,
                    self.finger_width,
                    self.hand_height,
                ]
            )
            approach_lwh = np.array([0.08, self.finger_width, self.hand_height])

            base = self.create_hand_base_marker(
                left_bottom,
                right_bottom,
                frame,
                0.02,
                self.hand_height,
                i,
                frame_id,
            )
            left_finger = self.create_finger_marker(
                left_center,
                frame,
                finger_lwh,
                i * 3,
                frame_id,
            )
            right_finger = self.create_finger_marker(
                right_center,
                frame,
                finger_lwh,
                i * 3 + 1,
                frame_id,
            )
            approach = self.create_finger_marker(
                approach_center,
                frame,
                approach_lwh,
                i * 3 + 2,
                frame_id,
            )

            marker_array.markers.extend(
                [
                    left_finger,
                    right_finger,
                    approach,
                    base,
                ]
            )

        return marker_array

    def create_finger_marker(self, center, frame, lwh, marker_id, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = "finger"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.lifetime = rospy.Duration(10)

        quat = tf.quaternion_from_matrix(frame)
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]

        marker.scale.x = lwh[0]  # forward direction
        marker.scale.y = lwh[1]  # hand closing direction
        marker.scale.z = lwh[2]  # hand vertical direction

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.5

        return marker

    def create_hand_base_marker(
        self, start, end, frame, length, height, marker_id, frame_id
    ):
        center = start + 0.5 * (end - start)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = "hand_base"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.lifetime = rospy.Duration(10)

        quat = tf.quaternion_from_matrix(frame)
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]

        marker.scale.x = length  # forward direction
        marker.scale.y = np.linalg.norm(end - start)  # hand closing direction
        marker.scale.z = height  # hand vertical direction

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        return marker
