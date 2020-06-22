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
from actionlib_msgs.msg import GoalStatusArray
from teb_local_planner.msg import OptimizationCostArray
from geometry_msgs.msg import Pose, Twist
import os
from hanp_msgs.msg import TrackedHumans
from hanp_msgs.msg import TrackedSegmentType
from hanp_msgs.msg import TrackedSegment


class PlotterHateb:
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
        self.ttg = []
        self.ttg_prev = 0.0
        self.ttg_current = 0.0
        self.done = False
        self.flag = True
        self.min_hum_dist = -1.0
        self.total_time = -1.0
        self.cost_ttc = 0
        self.ttc_arr = []
        self.robot_vel_x = []
        self.robot_vel_y = []
        self.robot_vel_theta = []

        self.current_pose = Pose()
        self.current_vel = Twist()

        self.human_pose = Pose()
        self.human_vel = Twist()


        self.fig = plt.figure()
        self.fig2 = plt.figure()

        # self.ax = self.fig.add_subplot(4,3,1)

        plt.ion()
        rospy.init_node('plotter_hateb')
        rospy.Rate(10)

        # Use this for HATEB
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,self.pathCB)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)
        rospy.Subscriber("/move_base/status",GoalStatusArray,self.totalTimeCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/Robot_Pose",Pose,self.poseCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/robot_vel",Twist,self.velCB,queue_size=1)
        rospy.Subscriber("/tracked_humans",TrackedHumans,self.humansCB)

        self.ttg_pub = rospy.Publisher('ttg', Float64, queue_size=1)
        self.ttg_prev_pub = rospy.Publisher('ttg_diff_prev', Float64, queue_size=1)
        self.ttg_first_pub = rospy.Publisher('ttg_diff_start', Float64, queue_size=1)
        self.total_time_pub = rospy.Publisher('total_time', Float64, queue_size=1)

        self.listener = tf.TransformListener()
        self.last_time = rospy.Time.now()
        self.begin_time = rospy.Time.now()

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
            self.robot_vel_x = []
            self.robot_vel_y = []
            self.robot_vel_theta = []
            self.first_x = []
            self.first_y = []
            for pose_it in self.first_path.poses:
                self.first_x.append(pose_it.pose.position.x)
                self.first_y.append(pose_it.pose.position.y)

        else:
            self.current_path = msg

    def poseCB(self, msg):
        (r,p,y) = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.actual_position_x.append(msg.position.x)
        self.actual_position_y.append(msg.position.y)
        self.actual_position_theta.append(y)
        self.current_pose = msg


    def ttgCB(self,msg):
        if not self.done:
            self.ttg_error = []
            self.ttg = []
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
        self.ttg.append(self.ttg_current)

    def totalTimeCB(self,msg):
        # rospy.get_param("/move_base/status/buffer_size")
        if(msg.status_list):
            if(msg.status_list[len(msg.status_list)-1].status == 1):
                if(self.flag):
                    self.begin_time = rospy.Time.now()
                self.flag = False

            elif(msg.status_list[len(msg.status_list)-1].status == 3 and (not self.flag) ):
                self.flag = True
                self.total_time_pub.publish((rospy.Time.now() - self.begin_time).to_sec())
                self.total_time = ((rospy.Time.now() - self.begin_time).to_sec())
                # self.save_data()
                self.cost_ttc = 0
                self.first_path.poses = []
                self.done = False
                self.costs_sum = OptimizationCostArray()


            else:
                self.total_time_pub.publish(-1.0)

    def velCB(self,msg):
        self.robot_vel_x.append(msg.linear.x)
        self.robot_vel_y.append(msg.linear.y)
        self.robot_vel_theta.append(msg.angular.z)
        self.current_vel = msg

    def humansCB(self, msg):
        for human in msg.humans:
            for segment in human.segments:
                if segment.type==TrackedSegmentType.TORSO:
                    self.human_pose = segment.pose.pose
                    self.human_vel = segment.twist.twist

    def plot_traj(self):
        plt.clf()

        plt.figure(1)
        if(len(self.current_path.poses)!=0):
            plt.plot(self.current_path.poses[0].pose.position.x, self.current_path.poses[0].pose.position.y,'r*')
            plt.plot(self.actual_position_x, self.actual_position_y,'b')
            # plt.axis('square')
            plt.axis('equal')
            plt.title('robot_path')

        plt.figure(2)
        if(len(self.current_path.poses)!=0):
            plt.subplot(3,1,1)
            plt.plot(self.robot_vel_x,'b')
            plt.title('velocity_x')
            plt.subplot(3,1,2)
            plt.plot(self.robot_vel_y,'b')
            plt.title('velocity_y')
            plt.subplot(3,1,3)
            plt.plot(self.robot_vel_theta,'b')
            plt.title('velocity_theta')
            plt.axis('equal')

        plt.pause(0.00001)
        plt.draw()
        plt.show()

    def cal_ttc(self):
        robot_radius = 0.34
        human_radius = 0.3
        radius_sum_ = robot_radius + human_radius
        radius_sum_sq_ = radius_sum_ * radius_sum_
        r_vel = self.current_vel.linear
        C = self.human_pose.pose.position - self.current_pose.pose.position
        ttc = np.inf
        C_sq = np.dot(C,C)

        if C_sq <= radius_sum_sq_:
            ttc = 0.0
        else:




    def run(self):
        while(1):
            self.plot_traj()
            time.sleep(0.1)


if __name__ == "__main__":
    plotter = PlotterHateb()
    plotter.run()
