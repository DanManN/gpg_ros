#!/usr/bin/env python3

import pygpg
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import transformations as tf
from geometry_msgs.msg import Pose, PoseStamped
from gpg_ros.msg import Grasps
from gpg_ros.srv import GetGrasps, GetGraspsResponse

from grasp_plotter import GraspPlotter



def list_to_pose(pose_list):
    pose_msg = Pose()
    pose_msg.position.x = pose_list[0]
    pose_msg.position.y = pose_list[1]
    pose_msg.position.z = pose_list[2]
    pose_msg.orientation.x = pose_list[3]
    pose_msg.orientation.y = pose_list[4]
    pose_msg.orientation.z = pose_list[5]
    pose_msg.orientation.w = pose_list[6]
    return pose_msg
 
def list_to_pose_stamped(pose_list, target_frame):
    pose_msg = PoseStamped()
    pose_msg.pose = list_to_pose(pose_list)
    pose_msg.header.frame_id = target_frame
    pose_msg.header.stamp = rospy.Time.now()
    return pose_msg

def matrix_to_pose(matrix):
    translation = list(tf.translation_from_matrix(matrix))
    quaternion = list(tf.quaternion_from_matrix(matrix))
    pose_list = translation + quaternion[1:] + quaternion[:1]
    # pose_list = translation + quaternion
    return list_to_pose(pose_list)

def matrix_to_pose_stamped(matrix, target_frame):
    translation = list(tf.translation_from_matrix(matrix))
    quaternion = list(tf.quaternion_from_matrix(matrix))
    pose_list = translation + quaternion[1:] + quaternion[:1]
    # pose_list = translation + quaternion
    return list_to_pose_stamped(pose_list, target_frame)

class GraspPlanner():

    def __init__(self):
        self.gripper_cfg = rospy.get_param('~config')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60))
        self.tf_listen = tf2_ros.TransformListener(self.tf_buffer)

        # TODO read gripper_cfg for gripper params
        self.hand_depth = 0.0613
        self.hand_height = 0.035
        self.grasp_plotter = GraspPlotter()

    def get_transform(self, target_frame, source_frame):
        if target_frame == source_frame:
            return None
        try:
            in2out = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(),
                rospy.Duration(1.0),
            )
            return in2out
        except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
        ):
            print(
                f"""
    Warning: Couldn't get transform from [{source_frame}] to [{target_frame}]!"
    Returning grasps in frame [{source_frame}]
                  """
            )
            return None

    def get_grasp_poses(
        self,
        points,
        input_frame,
        pre_grasp_dist=0.05,
        output_frame='world',
        num_samples=500,
        pub_rviz=True
    ):
        # get transform from input frame to output frame
        in2out = self.get_transform(output_frame, input_frame)

        grasps = pygpg.generate_grasps(
            points,
            num_samples,
            False,
            self.gripper_cfg,
        )
        if pub_rviz:
            self.grasp_plotter.draw_grasps(
                filter(lambda g: g.is_half_antipodal(), grasps),
                frame=input_frame
            )

        pose_list = []
        for grasp in grasps:
            if not grasp.is_half_antipodal():
                continue

            pose = np.eye(4)
            pose[:3, 0] = grasp.get_grasp_approach()
            pose[:3, 1] = grasp.get_grasp_binormal()
            pose[:3, 2] = grasp.get_grasp_axis()

            # displace position by pre_grasp_dist
            approach = pose[:3, 0] / np.linalg.norm(pose[:3, 0])
            displace = self.hand_height - pre_grasp_dist
            bottom = grasp.get_grasp_bottom() + displace * approach
            pose[:3, 3] = bottom

            #why do I need this transform?
            pose = np.matmul(
                pose,
                [
                    [0., 0., 1., 0.],
                    [0, -1., 0., 0.],
                    [1., 0., 0., 0.],
                    [0., 0., 0., 1.],
                ],
            )

            # transform into output_frame
            if in2out is not None:
                pose_stamped = matrix_to_pose_stamped(pose, input_frame)
                pose_transformed = tf2_geometry_msgs.do_transform_pose(
                    pose_stamped,
                    in2out,
                )
                pose_list.append(pose_transformed.pose)
            else:
                pose_list.append(matrix_to_pose(pose))
        return pose_list


    def handle_grasp_request(self, req):
        points = np.array([(p.x,p.y,p.z) for p in req.points])
        grasps = self.get_grasp_poses(points, req.frame_id)
        grasps_msg = Grasps()
        grasps_msg.header.frame_id = 'world'
        grasps_msg.grasps = grasps
        #float64[] grasp_scores

        return GetGraspsResponse(grasps_msg)

if __name__ == "__main__":
    rospy.init_node('gpg_grasp_server')
    print(rospy.get_param_names())
    grasp_planner = GraspPlanner()
    s = rospy.Service('get_grasps', GetGrasps, grasp_planner.handle_grasp_request)
    print("Ready to generate grasps...")
    rospy.spin()
