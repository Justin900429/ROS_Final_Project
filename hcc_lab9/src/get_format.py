#!/usr/bin/env python2
import time
import json
import rospy
import numpy as np
import message_filters
import cv2
import tf
from collections import OrderedDict
from tf import transformations as t
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo, Joy
from geometry_msgs.msg import PointStamped

# Init the node and set the name to anonymous
rospy.init_node('D435_Object_Distance', anonymous=True)
# Object for TransformListener and CvBridge
listener = tf.TransformListener()
cv_bridge = CvBridge()

rospy.loginfo("Start D435_Object_Distance")

print('Try to get camera info...')

msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=None)
 #     [fx'  0  cx' Tx]
 # P = [ 0  fy' cy' Ty]
 #     [ 0   0   1   0]
print('Get camera info')
fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]


def main():
    # Declare the global variable
    global result
    global count
# Set the init
    count = 1
    result = {"detection": []}


    # Using mesage filters to get multiple image
    depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
    target_image = message_filters.Subscriber('/darknet_ros/detection_image', Image)
    joy_sub = message_filters.Subscriber('/joy', Joy)

    # Synchronize the topic and throw into the callback
    ts = message_filters.ApproximateTimeSynchronizer([depth_image_sub, bb_sub, target_image, joy_sub], 1, 100)
    ts.registerCallback(callback)

    rospy.spin()

    # Write out the Json file
    with open("result.json", 'w') as output:
         json.dump(result, output)


def callback(depth_img, bb, output_image, joy_sub):
    global count

    if (joy_sub.buttons[1] == 1): # Write into file only if pressed "b"
        try:
            print("Detect to write")

            # Change the depth_image to proper format
            cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_img, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)

            # Change the image msg to the np.array
            result_img = cv_bridge.imgmsg_to_cv2(output_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Use OrderDict to remain the expected order
        temp_result = OrderedDict()
        temp_result["main_class"] = None
        temp_result["x"] = None
        temp_result["y"] = None
        temp_result["z"] = None
        temp_result["image_name"] = None
        temp_result["boxes"] = []

        try:
            # Write out the image
            cv2.imwrite("./images/{}.jpg".format(count), result_img)
            temp_result["image_name"] = "{}.jpg".format(count)
        except:
            pass


        # For knowing the additional info
        know = 1

        # Loop for the bounding boxes
        for i in bb.bounding_boxes:
            rospy.loginfo(i.Class) # print out the class

            if (know == 1):
                # Know the position of the object
                x_mean = (i.xmax + i.xmin) / 2
                y_mean = (i.ymax + i.ymin) / 2

                temp_result["main_class"] = i.Class

                # Transform depth image to distance
                zc = cv_depthimage2[int(y_mean)][int(x_mean)]
                v1 = np.array(getXYZ(x_mean, y_mean, zc, fx, fy, cx, cy))

                point_message = PointStamped()
                point_message.header = depth_img.header
                point_message.header.frame_id = "camera_color_optical_frame"
                point_message.point.x = v1[0]/1000
                point_message.point.y = v1[1]/1000
                point_message.point.z = v1[2]/1000


                # Set the time to real time
                now = rospy.Time(0)

                # Wait fot the transform form origin to camera_link
                try:
                    listener.waitForTransform("camera_link", "origin", now, rospy.Duration(10.0))
                except:
                    continue


                # Transform the position to the world of origin
                obj_point = point_message
                obj_point.header.stamp = rospy.Time(0)
                obj_point = listener.transformPoint("origin", obj_point)
                print(obj_point.point.x, obj_point.point.y, obj_point.point.z)

                # Save the output
                temp_result["x"] = obj_point.point.x
                temp_result["y"] = obj_point.point.y
                temp_result["z"] = obj_point.point.z

                know += 1

                # end for

            # For the bounding max message
            inner_dict = OrderedDict()
            inner_dict["class"] = i.Class
            inner_dict["top"] = i.ymin / 480.0
            inner_dict["bottom"] = i.ymax / 480.0
            inner_dict["left"] = i.xmin / 640.0
            inner_dict["right"] = i.xmax / 640.0

            temp_result["boxes"].append(inner_dict)

        # Write into to result dict
        result["detection"].append(temp_result)
        print("Finish writing {} iter".format(count))
        count += 1


def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    #### Definition:
    # cx, cy : image center(pixel)
    # fx, fy : focal length
    # xp, yp: index of the depth image
    # zc: depth
    inv_fx = 1.0/fx
    inv_fy = 1.0/fy
    x = (xp-cx) *  zc * inv_fx
    y = (yp-cy) *  zc * inv_fy
    z = zc
    return (x,y,z)


if __name__ == '__main__':
    main()
