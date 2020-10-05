#!/usr/bin/env python
import rospy
import numpy as np
# import math
# import tf
import roslib
# import matplotlib
import time
# from tf.transformations import euler_from_quaternion
# matplotlib.use('TKAgg')
# import matplotlib.pyplot as plt
# from nav_msgs.msg import Path
# from metrics.msg  import TimeToGoal
from std_msgs.msg import Float32
# from actionlib_msgs.msg import GoalStatusArray
from teb_local_planner.msg import OptimizationCostArray
# from geometry_msgs.msg import Pose, Twist
import os
# from hanp_msgs.msg import TrackedHumans
# from hanp_msgs.msg import TrackedSegmentType
# from hanp_msgs.msg import TrackedSegment
import matplotlib.pyplot as plt
from copy import deepcopy


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

class Static_viz_graph():
    def __init__(self):
        self.opt_costs = {
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
                    "HUMAN_ROBOT_TTCplus": [],
                    "HUMAN_ROBOT_VISIBILITY" : [],
                    "HUMAN_ROBOT_DIR" : []
                }

        self.start = False


    def opCostCB(self,msg):
        cost_value = 0.0
        costs_array = []
        # print("opCostcb")
        # Reset the data
        self.opt_costs = {key:[] for key in self.opt_costs}
        # self.opt_costs = self.opt_costs.fromkeys(self.opt_costs, [])
        # print(msg.costs)
        for cost in msg.costs:
            if cost.type==HUMAN_ROBOT_TTCplus:
                cost_value = cost.cost
                costs_array = deepcopy(cost.costs_arr)
                self.opt_costs["HUMAN_ROBOT_TTCplus"].append([cost_value, costs_array])
            if cost.type==HUMAN_ROBOT_VISIBILITY:
                cost_value = cost.cost
                costs_array = deepcopy(cost.costs_arr)
                self.opt_costs["HUMAN_ROBOT_VISIBILITY"].append([cost_value, costs_array])
            if cost.type==HUMAN_ROBOT_SAFETY:
                cost_value = cost.cost
                costs_array = deepcopy(cost.costs_arr)
                self.opt_costs["HUMAN_ROBOT_SAFETY"].append([cost_value, costs_array])
            if cost.type==HUMAN_ROBOT_DIR:
                cost_value = cost.cost
                costs_array = deepcopy(cost.costs_arr)
                self.opt_costs["HUMAN_ROBOT_DIR"].append([cost_value, costs_array])
        self.start = True
        # self.data_plot()

    def static_plot_pub(self,event=None):
        # print("1")
        if self.start:
            # print("2")
            for i in range(0, len(self.opt_costs["HUMAN_ROBOT_TTCplus"][0][1])):
                # print("3")

                cost = self.opt_costs["HUMAN_ROBOT_TTCplus"][0][1][i]
                self.ttcplus_pub.publish(cost)
                time.sleep(1.0/40.0)
        self.opt_costs = {key:[] for key in self.opt_costs}
        # self.opt_costs = self.opt_costs.fromkeys(self.opt_costs, [])
        self.start = False;


    def run(self):
        rospy.init_node('Static_viz_plotter')
        rospy.Rate(40)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/optimization_costs",OptimizationCostArray,self.opCostCB,queue_size=1)

        self.ttcplus_pub = rospy.Publisher('ttcplus_cost', Float32, queue_size=1)
        self.visibility_pub = rospy.Publisher('visibility_cost', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/40.0), self.static_plot_pub)
        rospy.spin()

    def data_plot(self):
        n = len(self.opt_costs["HUMAN_ROBOT_TTCplus"]) # Anycost can be used here
        # print(self.opt_costs)
        plt.figure(1)
        if(len(self.opt_costs["HUMAN_ROBOT_VISIBILITY"]) is not 0):
            plt.plot(self.opt_costs["HUMAN_ROBOT_VISIBILITY"][n-1][1],'.-', label='visibility')
        if(len(self.opt_costs["HUMAN_ROBOT_DIR"]) is not 0):
            plt.plot(self.opt_costs["HUMAN_ROBOT_DIR"][n-1][1], '.-', label='rel_velocity')
        if(len(self.opt_costs["HUMAN_ROBOT_SAFETY"]) is not 0):
            plt.plot(self.opt_costs["HUMAN_ROBOT_SAFETY"][n-1][1], '.-', label='safety')
        if(len(self.opt_costs["HUMAN_ROBOT_TTCplus"]) is not 0):
            plt.plot(self.opt_costs["HUMAN_ROBOT_TTCplus"][n-1][1], '.-', label='ttcplus')


        plt.legend()
        plt.xlabel('Poses (number)')
        plt.ylabel('Costs')
        plt.show()




if __name__ == "__main__":
    plotter = Static_viz_graph()
    plotter.run()
