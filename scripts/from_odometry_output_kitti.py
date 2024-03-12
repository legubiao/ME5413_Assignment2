#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
import tf2_ros

counter = 0

def callback(data):
    global counter
    global f
    # Get pose from odometry
    pose = data.pose.pose
    # Retrieve position and orientation
    position = pose.position
    orientation = pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    # Convert quaternion to rotation matrix
    rotation_matrix = np.array(tf.transformations.quaternion_matrix(quaternion)[0:3, 0:3])
    # Extract the translation vector
    translation_vector = np.array([position.x, position.y, position.z])
    # Get 4*4 matrix
    lidar_pose = np.zeros((4, 4))
    lidar_pose[:3, :3] = rotation_matrix
    lidar_pose[:3, 3] = translation_vector
    lidar_pose[3, 3] = 1
    # Transform the lidar pose to the camera frame
    camera_pose = np.dot(tfchange, lidar_pose)
    kitti_data = camera_pose[:3, :].flatten()
    # Format data to write output
    formatted_data = ["%.7e" % item for item in kitti_data]
    f.write(' '.join(formatted_data) + '\n')
    counter += 1
    sys.stdout.write("\rProcessed %i poses" % counter)
    sys.stdout.flush()


def listener():
    rospy.init_node('listener', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the transform to become available
    if not tf_buffer.can_transform('camera_gray_left', 'velo_link', rospy.Time(), timeout=rospy.Duration(5.0)):
        rospy.logerr('Failed to find transform.')
        return

    # Lookup the transform
    try:
        transform_stamped = tf_buffer.lookup_transform('camera_gray_left', 'velo_link', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr('Failed to find transform.')
        return

    rospy.loginfo("Successfully found transform: ")

    rospy.loginfo("Translation Vector: [%s, %s, %s]" % (transform_stamped.transform.translation.x,
                                                        transform_stamped.transform.translation.y,
                                                        transform_stamped.transform.translation.z))

    rospy.loginfo("Rotation Quaternion: [%s, %s, %s, %s]" % (transform_stamped.transform.rotation.x,
                                                             transform_stamped.transform.rotation.y,
                                                             transform_stamped.transform.rotation.z,
                                                             transform_stamped.transform.rotation.w))
    # Convert rotation from Quaternion to 4x4 matrix
    transform_matrix = tf.transformations.quaternion_matrix([transform_stamped.transform.rotation.x,
                                                             transform_stamped.transform.rotation.y,
                                                             transform_stamped.transform.rotation.z,
                                                             transform_stamped.transform.rotation.w])
    # Insert translations into 4x4 matrix
    transform_matrix[0, 3] = transform_stamped.transform.translation.x
    transform_matrix[1, 3] = transform_stamped.transform.translation.y
    transform_matrix[2, 3] = transform_stamped.transform.translation.z

    rospy.loginfo("Resulting Transform Matrix: \n %s " % transform_matrix)

    global tfchange
    tfchange = transform_matrix

    # Subscribe to Odometry topic
    rospy.Subscriber("/aft_mapped_to_init_high_frec", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    global f
    # Open file to write output
    f = open("aloam_result.txt", "w")
    listener()
    f.close()