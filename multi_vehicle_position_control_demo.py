"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 0.5
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False
    # location
    orientation = quaternion_from_euler(0,0, 3.14/2+3.14/8)
    locations = numpy.matrix([[2, 0, 1, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [83.23, -54.411, 20.89, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [83.23, -54.411, 17.89, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [83.23, -54.411, 27.89, orientation[0], orientation[1], orientation[2], orientation[3]],

                              [-76, 425, 60, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [-76, 425, 1, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [0, 0, 0, 0, 0, 0, 0]
                              ])

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/uav1/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape
        is_attached = False
        while not rospy.is_shutdown():
            print self.sim_ctr, shape[0], self.waypointIndex

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.waypointIndex is 2:
                attach.publish("ATTACH")
                attach.publish("ATTACH")


            # if self.waypointIndex == 6:
            #     attach.publish("DETACH")

            if self.isReadyToFly:
                [des_x, des_y, des_z] = self.set_desired_pose()
                # azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                #                      self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                # quaternion = quaternion_from_euler(0, 0, azimuth)

                curr_x = self.curr_drone_pose.pose.position.x
                curr_y = self.curr_drone_pose.pose.position.y
                curr_z = self.curr_drone_pose.pose.position.z

                dist = math.sqrt(
                    (curr_x - des_x) * (curr_x - des_x) + (curr_y - des_y) * (curr_y - des_y) + (curr_z - des_z) * (
                                curr_z - des_z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = min(self.curr_rover_pose.pose.position.y, 60)
        return self.des_pose.pose.position.x, self.des_pose.pose.position.y, self.des_pose.pose.position.z

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()
