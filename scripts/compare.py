#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
import roslib
import matplotlib
import time
from tf.transformations import euler_from_quaternion
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from metrics.msg  import TimeToGoal
from std_msgs.msg import Float64

class CompareTrajs:
    def __init__(self):
        self.first_path = Path()
        self.first_ttg = TimeToGoal()
        self.current_path = Path()
        self.previous_path = Path()
        self.actual_position_x = []
        self.actual_position_y = []
        self.actual_position_theta = []
        self.first_x = []
        self.first_y = []
        self.ttg_error = []
        self.ttg_prev = 0.0
        self.ttg_current = 0.0
        self.done = False

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        plt.ion()
        rospy.init_node('compare_trajs')
        try:
            rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,self.pathCB)
            rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)
        except:
            rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan",Path,self.pathCB)
            rospy.Subscriber("/move_base/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)
        else:
            rospy.logerr("Node Name Error..Please check the name of MoveBase node")
            print("Node Name Error..Please check the name of MoveBase node")


        self.ttg_pub = rospy.Publisher('ttg', Float64, queue_size=10)
        self.ttg_prev_pub = rospy.Publisher('ttg_diff_prev', Float64, queue_size=10)
        self.ttg_first_pub = rospy.Publisher('ttg_diff_start', Float64, queue_size=10)

        self.listener = tf.TransformListener()
        self.last_time = rospy.Time.now()

        # rospy.spin()

    def pathCB(self,msg):
        # print(self.first_path)
        self.last_time = rospy.Time.now()
        if len(self.first_path.poses)==0:
            # self.done = False
            self.first_path = msg
            self.actual_position_x = []
            self.actual_position_y = []
            self.actual_position_theta = []
            self.first_x = []
            self.first_y = []
            for pose_it in self.first_path.poses:
                self.first_x.append(pose_it.pose.position.x)
                self.first_y.append(pose_it.pose.position.y)

        else:
            self.current_path = msg

        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        (r,p,y) = euler_from_quaternion(rot)

        self.actual_position_x.append(trans[0])
        self.actual_position_y.append(trans[1])
        self.actual_position_theta.append(y)

    def ttgCB(self,msg):
        if not self.done:
            self.ttg_error = []
            self.first_ttg = msg.time_to_goal.to_sec()
            self.done = True
            self.ttg_prev = msg.time_to_goal.to_sec()
            self.ttg_current = self.ttg_prev

        # print((msg.time_to_goal - self.first_ttg).to_sec())
        else:
            self.ttg_prev = self.ttg_current
            self.ttg_current = msg.time_to_goal.to_sec()

        self.ttg_pub.publish(msg.time_to_goal.to_sec())
        self.ttg_prev_pub.publish(self.ttg_prev-self.ttg_current)
        self.ttg_first_pub.publish(self.first_ttg-self.ttg_current)

        self.ttg_error.append(self.ttg_prev-self.ttg_current)




    def plt_draw(self):
        self.ax.clear()
        self.ax.plot(self.first_x, self.first_y,'r')
        self.ax.plot(self.actual_position_x, self.actual_position_y,'b')
        plt.draw()
        plt.pause(0.05)

    def ttg_draw(self):
        self.ax.clear()
        self.ax.plot(self.ttg_error,'r')
        # self.ax.plot(self.first_x, self.first_y,'r')
        # self.ax.plot(self.actual_position_x, self.actual_position_y,'b')
        plt.draw()
        plt.pause(0.05)
        # plt.clf()




if __name__ == "__main__":
    compare = CompareTrajs()
    rospy.spin()
    #while(1):
    #    now = rospy.Time.now()
    #    if (now-compare.last_time).secs > 1.0:
    #        compare.first_path.poses = []
    #        compare.done = False

    #    compare.ttg_draw()
    #    plt.show()
    #    time.sleep(0.1)
