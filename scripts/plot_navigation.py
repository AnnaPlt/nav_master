#!/usr/bin/env python3

#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
from tf2_msgs.msg import TFMessage

import cv2
import numpy as np
import matplotlib.pyplot as plt

# Parametri mappa
pgm_map = 'home/braingear/anna_ws/map.pgm'
resolution = 0.025
origin = [-13.3, 2.8]
invert_y = True
invert_x = True

# Carico la mappa
map_img = cv2.imread(pgm_map, cv2.IMREAD_GRAYSCALE)

# Buffer TF globale
tf_buffer = None

# Liste per salvare i dati trasformati
goal_points = []
odom_points = []

def world_to_map_coords(x_world, y_world):
    mx = int((x_world - origin[0]) / resolution)
    my = int((y_world - origin[1]) / resolution)
    if invert_y:
        my = map_img.shape[0] - my
    if invert_x:
        mx = map_img.shape[1] - mx
    return my, mx

def transform_point(x, y, from_frame, to_frame):
    try:
        #trans = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        # Costruisco un PointStamped
        point_in = geometry_msgs.msg.PointStamped()
        point_in.header.frame_id = from_frame
        point_in.point.x = x
        point_in.point.y = y
        point_in.point.z = 0.0
        # Trasformo
        point_out = tf2_geometry_msgs.do_transform_point(point_in, first_transform)
        return point_out.point.x, point_out.point.y
    except Exception as e:
        rospy.logwarn("Transform failed: {}".format(e))
        return None, None

def callback_goal(msg):
    # Trasformo la pose dal frame originale al frame tag_1
    from_frame = msg.header.frame_id
    to_frame = 'tag_1'
    x = msg.pose.position.x
    y = msg.pose.position.y
    x_t, y_t = transform_point(x, y, from_frame, to_frame)
    if x_t is not None:
        goal_points.append((x_t, y_t))

def callback_odom(msg):
    from_frame = msg.header.frame_id
    to_frame = 'tag_1'
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_t, y_t = transform_point(x, y, from_frame, to_frame)
    if x_t is not None:
        odom_points.append((x_t, y_t))

def callback_tag(msg):
    # Puoi usarlo per debug, qui non serve per trasformare
    pass

def callback_tf(msg):
    global first_transform
    for transform in msg.transforms:
        if transform.child_frame_id == 'tag_1':
            first_transform = transform  # memorizza la prima che trovi
            break
    pass

def main():
    global tf_buffer
    rospy.init_node('plot', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/goal", geometry_msgs.msg.PoseStamped, callback_goal)
    rospy.Subscriber("/odometry/filtered", nav_msgs.msg.Odometry, callback_odom)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback_tag)
    rospy.Subscriber("/tf", TFMessage, callback_tf)

    rospy.loginfo("In ascolto dei topic...")

    rospy.spin()  # Rimani in ascolto finché non chiudi la bag o interrompi

    # Terminato: plotto tutto
    plt.figure(figsize=(8, 8))
    plt.imshow(map_img, cmap='gray')

    if len(goal_points) > 0:
        goal_pixels = [world_to_map_coords(x, y) for x, y in goal_points]
        ys, xs = zip(*goal_pixels)
        plt.scatter(xs, ys, c='red', label='Goals')

    if len(odom_points) > 0:
        odom_pixels = [world_to_map_coords(x, y) for x, y in odom_points]
        ys, xs = zip(*odom_pixels)
        plt.scatter(xs, ys, c='blue', label='Odometry')

    plt.legend()
    plt.title("Pose trasformate in frame tag_1 e proiettate sulla mappa")
    plt.show()

if __name__ == '__main__':
    import tf2_geometry_msgs  # Import necessario
    main()
