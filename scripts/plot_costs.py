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
#robot_pose_msg, op_costs, robot vel
# costs to look at 0, 1 , 3, 4, 5, 6, 7, 8, 9, 10, 12 or 17
#Cost types
TIME_OPTIMALITY=0
KINEMATIC_DD=1
KINEMATIC_CL=2
ROBOT_VEL=3
HUMAN_VEL=4
ROBOT_ACC=5
HUMAN_ACC=6
OBSTACLE=7
DYNAMIC_OBSTACLE=8
VIA_POINT=9
HUMAN_ROBOT_SAFETY=10
HUMAN_HUMAN_SAFETY=11
HUMAN_ROBOT_TTC=12
HUMAN_ROBOT_DIR=13
HUMAN_ROBOT_MIN_DIST=14
HUMAN_ROBOT_VISIBILITY=15
HUMAN_ROBOT_TTClosest=16
HUMAN_ROBOT_TTCplus=17

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

        self.opcosts = []
        self.costs_sum = OptimizationCostArray()
        self.tmp_costs = {
                    "TIME_OPTIMALITY" : [],
                    "KINEMATIC_DD" : [],
                    "ROBOT_VEL" : [],
                    "HUMAN_VEL" : [],
                    "ROBOT_ACC": [],
                    "HUMAN_ACC": [],
                    "OBSTACLE": [],
                    "DYNAMIC_OBSTACLE": [],
                    "VIA_POINT" : [],
                    "HUMAN_ROBOT_SAFETY": [],
                    "HUMAN_ROBOT_TTC": [],
                    "HUMAN_ROBOT_TTCplus": []
                }


        self.fig = plt.figure()
        self.fig2 = plt.figure()

        # self.ax = self.fig.add_subplot(4,3,1)

        plt.ion()
        rospy.init_node('plotter_hateb_costs')
        rospy.Rate(10)

        # Use this for HATEB
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,self.pathCB)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/min_dist_human",Float64,self.minDistCB)
        rospy.Subscriber("/move_base/status",GoalStatusArray,self.totalTimeCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/optimization_costs",OptimizationCostArray,self.opCostCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/Robot_Pose",Pose,self.poseCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/robot_vel",Twist,self.velCB,queue_size=1)


        # Use this for S-TEB
        # rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan",Path,self.pathCB)
        # rospy.Subscriber("/move_base/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)

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

        # Use this when the robot pose is not being published
        # (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        # (r,p,y) = euler_from_quaternion(rot)
        #
        # self.actual_position_x.append(trans[0])
        # self.actual_position_y.append(trans[1])
        # self.actual_position_theta.append(y)

        (r,p,y) = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.actual_position_x.append(msg.position.x)
        self.actual_position_y.append(msg.position.y)
        self.actual_position_theta.append(y)


    def ttgCB(self,msg):
        if not self.done:
            self.ttg_error = []
            self.ttg = []
            self.first_ttg = msg.time_to_goal.to_sec()
            self.done = True
            self.ttg_prev = msg.time_to_goal.to_sec()
            self.ttg_current = self.ttg_prev
k
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

    def minDistCB(self,msg):
        self.min_hum_dist = msg.data

    def velCB(self,msg):
        self.robot_vel_x.append(msg.linear.x)
        self.robot_vel_y.append(msg.linear.y)
        self.robot_vel_theta.append(msg.angular.z)

    def opCostCB(self,msg):
        if(len(self.costs_sum.costs)==0):
            self.costs_sum = msg
        else:
            for i in range(0,len(msg.costs)):
                self.costs_sum.costs[i].cost += msg.costs[i].cost

        self.opcosts.append(msg)
        for cost in msg.costs:
            # print('costs=',cost)
            if(cost.type==TIME_OPTIMALITY):
                self.tmp_costs["TIME_OPTIMALITY"].append(cost.cost)
            if(cost.type==KINEMATIC_DD):
                self.tmp_costs["KINEMATIC_DD"].append(cost.cost)
            if(cost.type==ROBOT_VEL):
                self.tmp_costs["ROBOT_VEL"].append(cost.cost)
            if(cost.type==HUMAN_VEL):
                self.tmp_costs["HUMAN_VEL"].append(cost.cost)
            if(cost.type==ROBOT_ACC):
                self.tmp_costs["ROBOT_ACC"].append(cost.cost)
            if(cost.type==HUMAN_ACC):
                self.tmp_costs["HUMAN_ACC"].append(cost.cost)
            if(cost.type==OBSTACLE):
                self.tmp_costs["OBSTACLE"].append(cost.cost)
            if(cost.type==DYNAMIC_OBSTACLE):
                self.tmp_costs["DYNAMIC_OBSTACLE"].append(cost.cost)
            if(cost.type==VIA_POINT):
                self.tmp_costs["VIA_POINT"].append(cost.cost)
            if(cost.type==HUMAN_ROBOT_SAFETY):
                self.tmp_costs["HUMAN_ROBOT_SAFETY"].append(cost.cost)
            if(cost.type==HUMAN_ROBOT_TTC):
                self.tmp_costs["HUMAN_ROBOT_TTC"].append(cost.cost)
            if(cost.type==HUMAN_ROBOT_TTCplus):
                self.tmp_costs["HUMAN_ROBOT_TTCplus"].append(cost.cost)

    def plot_costs_op(self):
        plt.clf()

        plt.figure(1)
        plt.subplot(3,2,1)
        plt.plot(self.tmp_costs["TIME_OPTIMALITY"],'r')
        plt.title("TIME_OPTIMALITY ROBOT")

        plt.subplot(3,2,2)
        plt.plot(self.tmp_costs["KINEMATIC_DD"],'r')
        plt.title("KINEMATIC_DD")

        plt.subplot(3,2,3)
        plt.plot(self.tmp_costs["ROBOT_VEL"],'r')
        plt.title("ROBOT_VEL")

        plt.subplot(3,2,4)
        plt.plot(self.tmp_costs["HUMAN_VEL"],'b')
        plt.title("HUMAN_VEL")

        plt.subplot(3,2,5)
        plt.plot(self.tmp_costs["ROBOT_ACC"],'r')
        plt.title("ROBOT_ACC")

        plt.subplot(3,2,6)
        plt.plot(self.tmp_costs["HUMAN_ACC"],'b')
        plt.title("HUMAN_ACC")

        plt.figure(2)
        plt.subplot(3,2,1)
        plt.plot(self.tmp_costs["OBSTACLE"],'r')
        plt.title("OBSTACLE")

        plt.subplot(3,2,2) # Nothing
        plt.plot(self.tmp_costs["DYNAMIC_OBSTACLE"],'r')
        plt.title("DYNAMIC_OBSTACLE")

        plt.subplot(3,2,3)
        plt.plot(self.tmp_costs["VIA_POINT"],'r')
        plt.title("VIA_POINT")

        plt.subplot(3,2,4)
        plt.plot(self.tmp_costs["HUMAN_ROBOT_SAFETY"],'r')
        plt.title("HUMAN_ROBOT_SAFETY")

        plt.subplot(3,2,5) #Nothing when TTCplus is used
        plt.plot(self.tmp_costs["HUMAN_ROBOT_TTC"],'r')
        plt.title("HUMAN_ROBOT_TTC")

        plt.subplot(3,2,6) #Nothing when TTC is used
        plt.plot(self.tmp_costs["HUMAN_ROBOT_TTCplus"],'r')
        plt.title("HUMAN_ROBOT_TTCplus")

        # plt.figure(3)
        # if(len(self.current_path.poses)!=0):
        #     plt.plot(self.current_path.poses[0].pose.position.x, self.current_path.poses[0].pose.position.y,'r*')
        #     plt.plot(self.actual_position_x, self.actual_position_y,'b')
        #     # plt.axis('square')
        #     plt.axis('equal')
        #     plt.title('robot_path')
        #
        # plt.figure(4)
        # if(len(self.current_path.poses)!=0):
        #     plt.subplot(3,1,1)
        #     plt.plot(self.robot_vel_x,'b')
        #     plt.title('velocity_x')
        #     plt.subplot(3,1,2)
        #     plt.plot(self.robot_vel_y,'b')
        #     plt.title('velocity_y')
        #     plt.subplot(3,1,3)
        #     plt.plot(self.robot_vel_theta,'b')
        #     plt.title('velocity_theta')
        #     plt.axis('equal')

        plt.pause(0.05)
        plt.draw()
        # self.fig.tight_layout(pad=3.0)
        plt.show()


    def run(self):
        while(1):
            self.plot_costs_op()
            time.sleep(0.1)
        # rospy.spin()




if __name__ == "__main__":
    plotter = PlotterHateb()
    plotter.run()

    # For running just ros
    # rospy.spin()

    ## For drawing
    #while(1):
    #    now = rospy.Time.now()
    #    if (now-compare.last_time).secs > 1.0:
    #        compare.first_path.poses = []
    #        compare.done = False

    #    compare.ttg_draw()
    #    plt.show()
    #    time.sleep(0.1)
