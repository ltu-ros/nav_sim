#!/usr/bin/env python
import rospy

import tf
from nav_msgs.msg import Odometry

ran_cb = False

def odomCB(msg):
    global ran_cb
    if not ran_cb:
        rospy.loginfo("Publishing transforms...")
        ran_cb = True

    pos = msg.pose.pose.position
    r = msg.pose.pose.orientation
    # rospy.loginfo('Quat: {}'.format(r))
    br.sendTransform((pos.x, pos.y, pos.z),
                     (r.x, r.y, r.z, r.w),
                     msg.header.stamp,
                     msg.child_frame_id, msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('transform_broadcaster')
    odom_topic = rospy.get_param('~odom_topic')

    br = tf.TransformBroadcaster()
    rospy.Subscriber(odom_topic, Odometry, odomCB)
    rospy.spin()
