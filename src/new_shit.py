#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_matrix
import sys
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from tf.transformations import quaternion_from_matrix
import copy
import moveit_commander
import moveit_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from moveit_commander import conversions
import open3d as o3d
from pyquaternion import Quaternion

try:
    from math import pi
except ImportError:
    pi = 3.141592653589793

from std_msgs.msg import String


moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
arm_group = moveit_commander.MoveGroupCommander(group_name)

class DataProcessor():
    def __init__(self):
        self.points = None
        self.normals = None

    def nor_callback(self, data):
        points = np.asarray(list(pc2.read_points(data, skip_nans=True)))
        self.normals = points[:, :3]
        waypoints = self.visualize_normals()

        waypoints = np.asarray(waypoints)

        height = 51
        width = 43

        try:
            way_points = waypoints.reshape(width, height, -1)
            # print("way_points shape:", way_points.shape)
            # way_points = way_points.tolist()

        except ValueError as e:
            print(f"Error reshaping the array: {e}")
            return

        points_to_be_followed = []
        grid_to_be_followed = []
        toggle = False
        path_following = True

        # Define grid waypoints if required
        if path_following:
            for j in np.arange(0, height):

                line_to_be_followed = []

                for i in np.arange(0, width):
                    line_to_be_followed.append(way_points[i, j])
                    points_to_be_followed.append(way_points[i, j])

                if toggle:
                    line_to_be_followed = line_to_be_followed[::-1]
                    toggle = False

                else:
                    toggle = True
                    
                grid_to_be_followed.append(line_to_be_followed)
            
            points_to_follow = np.array(points_to_be_followed)
            points_to_follow = list(points_to_follow.reshape(width * height, -1))

            flattened_list = [item for sublist in points_to_follow for item in (sublist if isinstance(sublist, list) else [sublist])]

            # print(f"shape:{points_to_follow.shape}")
            # print(points_to_be_followed)

            # self.static_tf_broadcast(points_to_be_followed)

            # cartesian_plan, fraction = self.plan_cartesian_path(points_to_be_followed)
            # self.execute_plan(cartesian_plan)

            static_transforms = []
            for i, pose_goal in enumerate(flattened_list):
                # print(pose_goal)
                translation = pose_goal[:3]  # Get the first three elements for translation
                rotation = pose_goal[3:]  # Get the last four elements for rotation

                static_transforms.append({
                    'parent_frame_id': 'royale_camera_0_optical_frame',
                    'child_frame_id': 'frame_{}'.format(i),
                    'translation': translation,
                    'rotation': rotation
                })

            # Create a static transform broadcaster
            static_broadcaster = tf2_ros.StaticTransformBroadcaster()
            rate = rospy.Rate(10)
            t_list = []   
            for static_transform in static_transforms:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = static_transform['parent_frame_id']
                t.child_frame_id = static_transform['child_frame_id']
                t.transform.translation.x = static_transform['translation'][0]
                t.transform.translation.y = static_transform['translation'][1]
                t.transform.translation.z = static_transform['translation'][2]
                t.transform.rotation.x = static_transform['rotation'][0]
                t.transform.rotation.y = static_transform['rotation'][1]
                t.transform.rotation.z = static_transform['rotation'][2]
                t.transform.rotation.w = static_transform['rotation'][3]
                t_list.append(t)
            static_broadcaster.sendTransform(t_list)
            pose1 = Pose()
            pose2 = Pose()
            pose3 = Pose()
            pose4 = Pose()

            pose1.position.x = 0.208854
            pose1.position.y = -0.0567724
            pose1.position.z = 0.331463
            pose1.orientation.x = 0.98901
            pose1.orientation.y = 0.136671
            pose1.orientation.z = -0.0394402
            pose1.orientation.w = 0.403051

            # arm_group.set_pose_target(pose1)
            # success = arm_group.go(wait=True)

            for point in flattened_list:
                pose = Pose()
                pose.position.x = point.position.x
                pose.position.y = point.position.y
                pose.position.z = point.position.z
                pose.orientation.x = point.orientation.x
                pose.orientation.y = point.orientation.y
                pose.orientation.z = point.orientation.z
                pose.orientation.w = point.orientation.w
                arm_group.set_pose_target(pose1)
                success = arm_group.go(wait=True)

    def pc_callback(self, cloud_msg):
        points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))
        self.points = points
        # print(points.shape)

    def quaternion_from_normal(self, normal):
        # Assuming that the initial orientation is pointing upwards (e.g., [0, 0, 1])
        up_vector = np.array([0.0, 0.0, 1.0])
        
        # Compute the rotation axis using cross product
        axis = np.cross(up_vector, normal)
        
        # Compute the rotation angle using dot product
        angle = np.arccos(np.dot(up_vector, normal))

        # if np.linalg.norm(normal) == 0:
        #     z_axis_local = [0.0, 0.0, 1.0]
        # else:
        #     z_axis_local = 1 * normal
        # y_axis_global = np.array([0, 1, 0]).astype(float)

        # x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
        # if np.linalg.norm(x_axis_local) != 0:
        #     x_axis_local /= np.linalg.norm(x_axis_local)
        # else:
        #     x_axis_local = [1, 0, 0]

        # y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)
        # if np.linalg.norm(y_axis_local) != 0:
        #     y_axis_local /= np.linalg.norm(y_axis_local)
        # else:
        #     y_axis_local = [0.0, 1.0, 0.0]

        # rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
        # quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

        # quaternion_data /= np.linalg.norm(quaternion_data)

        # Create a quaternion from the axis and angle
        quaternion = quaternion_from_matrix([[np.cos(angle/2), -np.sin(angle/2)*axis[0], -np.sin(angle/2)*axis[1], -np.sin(angle/2)*axis[2]],
                                            [np.sin(angle/2)*axis[0], np.cos(angle/2), 0, 0],
                                            [np.sin(angle/2)*axis[1], 0, np.cos(angle/2), 0],
                                            [np.sin(angle/2)*axis[2], 0, 0, np.cos(angle/2)]])
        
        quaternion /= np.linalg.norm(quaternion)
        # return quaternion_data
        return quaternion

    def visualize_normals(self):

        waypoints_list = []
        # a = self.points is not None
        # b = self.normals is not None
        # print(f"a: {a}\nb: {b}")
        if self.points is not None and self.normals is not None:
            # Determine the minimum length between points and normals
            min_length = min(len(self.points), len(self.normals))

            # Create a MarkerArray to store arrows
            marker_array = MarkerArray()

            for i in range(min_length):
                point = self.points[i]
                normal = self.normals[i]
                # print(normal)
                # Compute quaternion from the normal vector
                quaternion = self.quaternion_from_normal(normal)

                pose_goal = Pose()
                pose_goal.position.x = point[0]
                pose_goal.position.y = point[1]
                pose_goal.position.z = point[2]
                pose_goal.orientation.x = quaternion[0]
                pose_goal.orientation.y = quaternion[1]
                pose_goal.orientation.z = quaternion[2]
                pose_goal.orientation.w = quaternion[3]

                waypoints_list.append(pose_goal)
                
                # Create a marker for each normal arrow
                marker = Marker()
                marker.header.frame_id = "royale_camera_0_optical_frame"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "normals"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                
                # Set the orientation using the computed quaternion
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]

                marker.scale.x = 0.01  # Arrow shaft diameter
                marker.scale.y = 0.001  # Arrow head diameter
                marker.scale.z = 0.001  # Arrow head length
                marker.color.a = 1.0  # Alpha (transparency)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

            # Publish the MarkerArray
            marker_pub.publish(marker_array)

        return waypoints_list

    def plan_cartesian_path(self, points_to_follow):
        # print("Entered cartesian plan function")
        plan, fraction = arm_group.compute_cartesian_path(
            points_to_follow, 
            0.01, 
            0.1, 
            avoid_collisions=True)
        
        return plan, fraction
    
    def execute_plan(self, cartesian_plan):
        arm_group.execute(cartesian_plan, wait = True)

        arm_group.stop()
        arm_group.clear_pose_targets()

            
def listener():
    rospy.init_node('visualize_normals', anonymous=True)

    data_processor = DataProcessor()

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, data_processor.pc_callback)

    rospy.Subscriber('/cloud_normals_something', PointCloud2, data_processor.nor_callback)

    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=10)
    listener()
