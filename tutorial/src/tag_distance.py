#!/usr/bin/env python

''' 
    Find the transform between tag_0 and tag_1, then calculate the distance between them
    Last edited: 10/1, 2018
    Editor: Sean Lu
'''

import rospy
import tf
from math import sqrt

if __name__ == "__main__":
	rospy.init_node("tag_distance_node", anonymous = False)
	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			now = rospy.Time.now()
			listener.waitForTransform('tag_0', 'tag_1', now, rospy.Duration(3.0))
			(trans, rot) = listener.lookupTransform('tag_0', 'tag_1', now)
			distance = sqrt(trans[0]**2+trans[1]**2+trans[2]**2)
			rospy.loginfo("Distance: %f" %(distance))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, \
			tf.Exception):
			continue
		rate.sleep()
	rospy.loginfo("End process")
