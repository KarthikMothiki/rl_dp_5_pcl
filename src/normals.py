#!/usr/bin/env python3
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
normal_pub = rospy.Publisher('normals', PointCloud2, queue_size=1)

def cloud_callback(cloud_msg):
    global normal_pub

    # Convert PointCloud2 message to a numpy array
    point_cloud = np.array(list(point_cloud2.read_points(cloud_msg)))

    # Create an Open3D point cloud
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(point_cloud[:, :3])

    # Estimate normals using Open3D (CPU version)
    o3d_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

    # Get the normals as a numpy array
    normals = np.asarray(o3d_cloud.normals)

    # Create a new PointCloud2 message for the normals
    normal_msg = point_cloud2.create_cloud_xyz32(cloud_msg.header, normals)

    # Publish the normals
    normal_pub.publish(normal_msg)

def main():
    rospy.init_node('normal_estimation', anonymous=True)
    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, cloud_callback, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()
