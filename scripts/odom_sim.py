#!/usr/bin/env python

from math import sin, cos

#pylint: disable=import-error
import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, PointStamped, Point, PoseWithCovarianceStamped

class Vec():
    """
    A small vector class to make some of the movement math cleaner
    """

    def __init__(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th

    def __add__(self, other):
        if isinstance(other, Vec):
            return Vec(self.x + other.x, self.y + other.y, self.th + other.th)
        else:
            return Vec(self.x + other, self.y + other, self.th + other)

    def __mul__(self, other):
        if isinstance(other, Vec):
            return Vec(self.x * other.x, self.y * other.y, self.th * other.th)
        else:
            return Vec(self.x * other, self.y * other, self.th * other)

    def toPoint(self):
        return Point(self.x, self.y, 0)

    def toQuat(self):
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        return Quaternion(*q)

    def toTwist(self):
        t = Twist()
        t.linear.x = self.x
        t.linear.y = self.y
        t.angular.z = self.th
        return t



class Robot:

    def __init__(self, x, y, name, speed=1.0):
        self.loc = Odometry()
        self.loc.header.frame_id = 'map'
        self.loc.child_frame_id = name
        self.vel = Vec(0,0,0)
        self.pos = Vec(x,y,0)
        self.speed = speed

    def setVel(self, twist):
        self.vel = Vec(twist.linear.x,
                       twist.linear.y,
                       twist.angular.z) * self.speed

    def decelerate(self, amount):
        self.vel = self.vel * amount

    def setPose(self, pose):
        self.setPosition(pose.position)
        self.setOrientation(pose.orientation)

    def setPosition(self, point):
        self.pos = Vec(point.x, point.y, self.pos.th)

    def setOrientation(self, quat):
        q = [quat.x, quat.y, quat.z, quat.w]
        # returns roll, pitch, yaw. We only need yaw
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        self.pos.th = yaw


    def update(self, dt):
        # Compute delta
        # https://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
        d = Vec((self.vel.x * cos(self.pos.th) - self.vel.y * sin(self.pos.th)) * dt,
                (self.vel.x * sin(self.pos.th) + self.vel.y * cos(self.pos.th)) * dt,
                self.vel.th * dt)

        # Apply delta
        self.pos = self.pos + d

    def getOdom(self, stamp):
        self.loc.header.stamp = stamp
        self.loc.pose.pose.position = self.pos.toPoint()
        self.loc.pose.pose.orientation = self.pos.toQuat()
        self.loc.twist.twist = self.vel.toTwist() # Velocity
        return self.loc


if __name__ == '__main__':
    rospy.init_node('odom_sim', anonymous=True)

    # Load the name of the robot
    try:
        ugv_id = rospy.get_param('~ugv_id')
    except KeyError:
        raise ValueError('odom_sim.py requires "ugv_id" param')

    robot = Robot(-16, -40, name=ugv_id, speed=0.9)

    # Robot location publisher
    pub = rospy.Publisher('/{}/odom'.format(ugv_id), Odometry, queue_size=10)
    # cmd_vel subscriber
    sub = rospy.Subscriber('/{}/cmd_vel'.format(ugv_id), Twist, robot.setVel)
    # Location reset subscriber
    reset_point_sub = rospy.Subscriber('/{}/set_pose'.format(ugv_id), PoseWithCovarianceStamped, lambda x: robot.setPose(x.pose.pose))

    limiter = rospy.Rate(20)
    current_time = rospy.Time.now()
    prev_time = current_time

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        delta_t = (current_time - prev_time).to_sec()

        robot.update(delta_t)
        robot.decelerate(0.95)

        prev_time = current_time
        pub.publish( robot.getOdom(current_time))
        limiter.sleep()
