#!/usr/bin/env python3
import rospy
import sys
from sklearn.neighbors import NearestNeighbors
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from tf.transformations import quaternion_from_matrix
import tf2_ros
import time
import moveit_commander
import moveit_msgs.msg
import open3d as o3d


## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('generating_custom_trajectory', anonymous=True)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
tf_listener = tf2_ros.TransformListener(tf_buffer)

box_flag = False
path_following = True

# Set up the robot arm group and planning scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Name of the group can be found at the "/home/inspire-0/moveit_igus_ws/src/Moveit_Igus_RL_DP5/rl_dp_5/rl_dp_5_moveit/config/igus_robolink_rl_dp_5.srdf" file
group_name = 'rldp5'
arm_group = moveit_commander.MoveGroupCommander(group_name)
# arm_group.set_named_target("upright")
# plan1 = arm_group.go()
# rospy.sleep(5)

display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

planning_frame = arm_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

arm_group.set_goal_orientation_tolerance(0.1)

# Set the start state of the robot
arm_group.set_start_state_to_current_state()
arm_group.set_max_velocity_scaling_factor(0.05)

# Sometimes for debugging it is useful to print the entire state of the robot:
print("============ Printing robot state")
# print(robot.get_current_state())
print("")

start_time = time.time()

# Global publisher for markers
global marker_pub

def visualize_trajectory(waypoints, r, g, b):
    """
    Visualizes a trajectory using markers.

    Parameters:
        waypoints (list): List of Pose points representing the trajectory.
        r (float): Red component of the RGB color.
        g (float): Green component of the RGB color.
        b (float): Blue component of the RGB color.

    Returns:
        marker_array (MarkerArray): Visualization of the trajectory as a MarkerArray.
    """

    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        # marker.header.frame_id = "tool_link_ee"
        marker.header.frame_id = "royale_camera_0_optical_frame"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.position.x
        marker.pose.position.y = point.position.y
        marker.pose.position.z = point.position.z
        marker.pose.orientation.x = point.orientation.x
        marker.pose.orientation.y = point.orientation.y
        marker.pose.orientation.z = point.orientation.z
        marker.pose.orientation.w = point.orientation.w
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_array.markers.append(marker)
        # print(" enumerating at {}".format(i))
    return marker_array

def visualize_normals(normal_vectors, point_array):
    """
    Visualizes normals as markers in Rviz.

    Args:
        normal_vectors (numpy.ndarray): Array of tangent vectors representing normals.
        point_array (numpy.ndarray): Array of 3D points corresponding to the point cloud.

    Returns:
        None

    Frame:
        Setting the frame to royale_camera_0_optical_frame is important because, 
        similar to our robot's TF, royale_camera_0_optical_frame is base_link and royale_camera_link is the world frame
    """
    marker_array = MarkerArray()

    for i in range(len(point_array)):
        normal_marker = Marker()
        normal_marker.header.frame_id = "royale_camera_0_optical_frame"
        normal_marker.header.stamp = rospy.Time.now()
        normal_marker.id = i
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD

        position_np = point_array[i]
        normal_np = normal_vectors[i]

        normal_marker.pose.position.x = position_np[0]
        normal_marker.pose.position.y = position_np[1]
        normal_marker.pose.position.z = position_np[2]

        # print(f"Marker_{i} position: \n{normal_marker.pose.position}")

        # Check for zero-length tangent vectors before normalization
        if np.linalg.norm(normal_np) != 0:
            normal_np /= np.linalg.norm(normal_np)

            z_axis_local = 1 * normal_np
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)

            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local /= np.linalg.norm(y_axis_local)
            z_axis_local /= np.linalg.norm(z_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)

            normal_marker.pose.orientation.x = quaternion_data[0]
            normal_marker.pose.orientation.y = quaternion_data[1]
            normal_marker.pose.orientation.z = quaternion_data[2]
            normal_marker.pose.orientation.w = quaternion_data[3]
        else:
            # If the tangent vector is zero, set an arbitrary orientation
            normal_marker.pose.orientation.w = 1.0

        normal_marker.scale.x = 0.01
        normal_marker.scale.y = 0.001
        normal_marker.scale.z = 0.001
        normal_marker.color.a = 1.0
        normal_marker.color.r = 1.0
        normal_marker.color.g = 1.0
        normal_marker.color.b = 0.0

        marker_array.markers.append(normal_marker)

    marker_pub.publish(marker_array)

def comp_normals(points):
    """
    Computes tangent vectors and normalizes them to obtain normals.

    Args:
        points (numpy.ndarray): Array of 3D points.

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Description:
        - The function compute_normals takes an array of 3D points (points) as input.
        - 'normal_vectors' is calculated by taking the difference between consecutive points. 
        - np.roll is used to shift the array by one position to get the differences. 
        - This is a numerical approximation of the derivative of the points, representing tangent vectors along the curve.
        - Special handling is applied to the first and last points to ensure that tangent vectors are computed properly.
    """
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(points)


    # Create an Open3D point cloud
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

    # Estimate normals using Open3D (CPU version)
    o3d_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

        # Get the normals as a numpy array
    selected_pcl_normals = np.asarray(o3d_cloud.normals)

    return selected_pcl_normals

def calc_normals(points, k=20):
    normals = []
    neigh = NearestNeighbors(n_neighbors=k)
    neigh.fit(points)
    avg_num_neighbors = np.mean([len(neigh.kneighbors([p], return_distance=False)[0]) for p in points])

    for p in points:
        indices = neigh.kneighbors([p], return_distance=False)[0]
        # print(indices)
        # if len(indices) < avg_num_neighbors:
        #     # Skip points with less than the average number of neighbors
        #     continue
        neighbors = points[indices]
        centroid = np.mean(neighbors, axis=0)
        covariance_matrix = np.cov(neighbors, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        if np.dot(normal, np.array([1,0,0])) < 0:
            normal *= 1
        normals.append(normal)
    return np.array(normals)

def compute_normals(points, epsilon=1e-3):
    """
    Compute normals for a point cloud using numerical differentiation.

    Parameters:
        points (numpy.ndarray): Array of 3D points representing the point cloud.
        epsilon (float): Step size for numerical differentiation.

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Description:
        This function estimates normals for each point in the input point cloud using
        numerical differentiation. It approximates the tangent vector at each point
        and normalizes it to obtain the normal vector.

    Args:
        points (numpy.ndarray): Array of 3D points.
        epsilon (float, optional): Step size for numerical differentiation (default is 1e-3).

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Note:
        - The function uses a simple numerical differentiation approach by selecting
          three neighboring points for each point in the point cloud.
        - The `epsilon` parameter controls the step size in numerical differentiation.
    """
    normals = []

    for i in range(len(points)):
        # Select three points for numerical differentiation
        p0 = points[max(0, i - 1)]
        p1 = points[i]
        p2 = points[min(len(points) - 1, i + 1)]

        # Numerical differentiation to estimate tangent vector
        tangent_vector = (p2 - p0) / (2 * epsilon)

        # Normalize tangent vector to obtain normal
        normal = tangent_vector / np.linalg.norm(tangent_vector)

        normals.append(normal)

    return np.array(normals)


# Function to move the robot to a specified pose goal
def go_to_pose_goal(pose_goal):
    """
    Moves the robot to a specified pose goal.

    Parameters:
        pose_goal (Pose): Target Pose for the end-effector.
        i (int): Index indicating the position in the trajectory (for logging purposes).
    """
    for i in range(len(pose_goal)):
    # print(f"Came into go_to_pose_goal function, values of pose_goal: {len(pose_goal)}")
        arm_group.set_pose_target(pose_goal[i])

        rospy.loginfo(f"Planning Trajectory to Pose {i}")
        plan=arm_group.plan()
        success = arm_group.go(wait=True)

        # rospy.loginfo(f"DONE EXECUTING Planning Trajectory success or failure : {success}")

        # current_joints = arm_group.get_current_joint_values()
        current_joints = arm_group.get_current_pose().pose
    # return all_close(pose_goal, current_joints, 0.01)

def display_trajectory(plan):
    """
    Displays a MoveIt trajectory.

    Parameters:
        plan (MoveIt plan): MoveIt plan to be displayed.
    """
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

def point_cloud_callback(cloud_msg):
    """
    Callback function to process incoming point cloud data.

    Args:
        cloud_msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
    """
    try:
        points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))

        # print(f"size: {points.shape}")
        height = 58
        width = 54

        # Ensure the array is 2D
        if len(points.shape) == 1:
            points = points.reshape(-1, 3)

        # Compute tangent vectors
        normal_vectors = compute_normals(points)
        # print(f"Normal Vectors: \n{normal_vectors}")

        print(f"Size of segmented point cloud: {points.shape}\nShape of normals computed: {normal_vectors.shape}")

        visualize_normals(normal_vectors, points)

        waypoints = []

        for i, normal in enumerate(normal_vectors):
            # print(f"Normal vector@{i}: {normal}\nPoint Vector@{i}: {points[i]}")
            z_axis_local = 1 * normal
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)
            y_axis_local /= np.linalg.norm(y_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)
            # print(f"Quaternion Data: \n{quaternion_data}")

            pose_goal =Pose()
            pose_goal.position.x = points[i][0]
            pose_goal.position.y = points[i][1]
            pose_goal.position.z = points[i][2]
            pose_goal.orientation.x = quaternion_data[0]
            pose_goal.orientation.y = quaternion_data[1]
            pose_goal.orientation.z = quaternion_data[2]
            pose_goal.orientation.w = quaternion_data[3]
            waypoints.append(pose_goal)
        
        print(f"Shape of way points: {np.array(waypoints).shape}")

        # print(f"\nWay points: \n{waypoints}\n")
        final_waypoints=np.array(waypoints).reshape(width, height, -1)
        # print(f"SHape: {final_waypoints.shape}")
        marker_pub_trajectory = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=1, latch=True)
                
        points_to_be_followed = []   
        box_waypoints = []

        box_flag = False

        # Define box waypoints if required
        if box_flag:
            for j in np.arange(0, height, 8):
                box_waypoints.append(final_waypoints[0, j, 0])            
                
            for i in np.arange(0, width, 8):
                box_waypoints.append(final_waypoints[i, height - 1, 0])

            for j in np.arange(0, height, 8)[::-1]:
                box_waypoints.append(final_waypoints[width - 1, j, 0])
                
            for i in np.arange(0, width, 8)[::-1]:
                box_waypoints.append(final_waypoints[i, 0, 0])
                
            points_to_be_followed = box_waypoints 
            print(f"Shape of points_to_be_followed: {np.array(points_to_be_followed).shape}")

        grid_to_be_followed = []   
        toggle = False

        # Define grid waypoints if required
        if path_following:
            for j in np.arange(0, height):

                line_to_be_followed = []

                for i in np.arange(0, width):
                    line_to_be_followed.append(final_waypoints[i, j, 0])
                    points_to_be_followed.append(final_waypoints[i, j, 0])

                if toggle:
                    line_to_be_followed = line_to_be_followed[::-1]
                    toggle = False

                else:
                    toggle = True
                    
                grid_to_be_followed.append(line_to_be_followed)

        print("jafdglo")
        print(f"Shape of grid_to_be_followed: {np.array(grid_to_be_followed).shape}")
                    
        # marker_array1 = visualize_trajectory(points_to_be_followed, 1.0, 2.0, 0.0)
        # marker_pub_trajectory.publish(marker_array1)
        
        # print("len(grid_to_be_followed):", len(grid_to_be_followed))
        # print(f"Shape of Points to be followed: {np.array(points_to_be_followed)}")
            
        static_transforms = []
        normal_points = points_to_be_followed

        for i, pose_goal in enumerate(normal_points):
            static_transforms.append({
                'parent_frame_id': 'royale_camera_0_optical_frame',
                # 'parent_frame_id': 'world',
                'child_frame_id': 'frame_{}'.format(i),
                'translation': [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z],
                'rotation': [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
                })

        print("Change the position if you want")
        input("******Press enter to display start trajectory ********")

        # print(f"Shape of box_waypoints: {np.array(box_waypoints).shape}")
        # print(f"Shape of grid_to_be_followed: {np.array(grid_to_be_followed).shape}")

        # print("z: ", normal_points[0].position.z)

        box_flag = True
        if box_flag:
            # for i, normal_point in enumerate(normal_points):
            #     normal_point.position.z += 0.2
                # print("ajdfl: ", normal_points[i].position.z)
            (plan, fraction) = arm_group.compute_cartesian_path(points_to_be_followed, 0.01, 0.1)

        else:
            grid_to_be_followed[0][0].position.z = grid_to_be_followed[0][0].position.z + 0.2

            (plan, fraction) = arm_group.compute_cartesian_path(
                [grid_to_be_followed[0][0]], 0.01, 0  # waypoints to follow  # eef_step
                )
            
        # print("Plan: ", plan)
            
        print(f"Grid to be followed: {len(grid_to_be_followed[1])}")
        print(f"Fraction: {fraction}")

        # input("******Press enter to execute trajectory ********")
        # print(f"normal_points[0]: {normal_points[0]}")

        # for i in range(len(normal_points)):
        if fraction < 0.1:
            go_to_pose_goal(normal_points)
        else:
            st = time.time()

            # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = plan.joint_trajectory
            velocity_scaling_factor = 0.05  # Adjust this value as desired
            retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
            print(f"Total time taken after retime: {st - time.time()}")

            arm_group.execute(retime_trajectory, wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        arm_group.stop()
        arm_group.clear_pose_targets()

        if box_flag:
            print("Display trajectory(cartesian_plan)")
            (plan, fraction) = arm_group.compute_cartesian_path(box_waypoints, 0.01, 0, avoid_collisions=True) 
            # waypoints to follow, eef_step, jump_threshold, avoid_collisions = True, path_constraints = None
            display_trajectory(plan)
            input("******Press enter to execute trajectory ********")
            st = time.time()

            # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = plan.joint_trajectory

            # Adjust the trajectory timing based on velocity scaling
            velocity_scaling_factor = 0.05  # Adjust this value as desired
            retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
            print(f"Total time taken after retime: {st - time.time()}")

            arm_group.execute(retime_trajectory, wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        else:
            input("******Press enter to execute trajectory one by one********")

            for i, waypoint_list in enumerate(grid_to_be_followed):
                (plan, fraction) = arm_group.compute_cartesian_path(waypoint_list, 0.01, 0.1)  # waypoints to follow  # eef_step

                input("******Press enter to display trajectory ******")
                display_trajectory(plan)
                print("Fraction for plan", i, ":", fraction)

                input("******Press enter to execute trajectory ******")
                st = time.time()

                # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
                robot_trajectory = RobotTrajectory()
                robot_trajectory.joint_trajectory = plan.joint_trajectory
                velocity_scaling_factor = 0.05  # Adjust this value as desired
                retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
                print(f"Total time taken after retime: {st - time.time()}")

                arm_group.execute(retime_trajectory, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()

            print("Finished code")

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

    

def main():
    global marker_pub

    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=1)

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, point_cloud_callback)

    rospy.spin()

if __name__ == "__main__":
    main()

    
