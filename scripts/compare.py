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
import os
#robot_pose_msg, op_costs

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
        self.ttg = []
        self.ttg_prev = 0.0
        self.ttg_current = 0.0
        self.done = False
        self.flag = True
        self.min_hum_dist = -1.0
        self.total_time = -1.0
        self.cost_ttc = 0
        self.ttc_arr = []


        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        plt.ion()
        rospy.init_node('compare_trajs')
        rospy.Rate(10)

        # Use this for HATEB
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,self.pathCB)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/min_dist_human",Float64,self.minDistCB)
        rospy.Subscriber("/move_base/status",GoalStatusArray,self.totalTimeCB,queue_size=1)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/optimization_costs",OptimizationCostArray,self.opCostCB,queue_size=1)


        # Use this for S-TEB
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan",Path,self.pathCB)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/traj_time",TimeToGoal,self.ttgCB)

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
            self.first_x = []
            self.first_y = []
            for pose_it in self.first_path.poses:
                self.first_x.append(pose_it.pose.position.x)
                self.first_y.append(pose_it.pose.position.y)

        else:
            self.current_path = msg

        (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        (r,p,y) = euler_from_quaternion(rot)

        self.actual_position_x.append(trans[0])
        self.actual_position_y.append(trans[1])
        self.actual_position_theta.append(y)

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
                self.save_data()
                self.cost_ttc = 0
                self.first_path.poses = []
                self.done = False


            else:
                self.total_time_pub.publish(-1.0)

    def minDistCB(self,msg):
        self.min_hum_dist = msg.data

    def opCostCB(self,msg):
        for cost in msg.costs:
            if(cost.type==17):
                self.cost_ttc+=cost.cost
                self.ttc_arr.append(cost.cost)


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

    def save_data(self):
        lt = time.asctime(time.localtime(time.time()))
        lt = ''.join( c for c in lt if  c not in ': ' )
        name = '/home/phani/hateb_data'

        # Depends on Experiment
        sub_name='door_new_map'

        dir1_path = name+'/initial_path/new_hateb/'+sub_name
        dir2_path = name+'/followed_path/new_hateb/'+sub_name
        dir3_path = name+'/ttgs/new_hateb/'+sub_name
        dir4_path = name+'/constants/new_hateb/'+sub_name

        if(not os.path.isdir(dir1_path)):
            os.makedirs(dir1_path)
        if(not os.path.isdir(dir2_path)):
            os.makedirs(dir2_path)
        if(not os.path.isdir(dir3_path)):
            os.makedirs(dir3_path)
        if(not os.path.isdir(dir4_path)):
            os.makedirs(dir4_path)

        if(self.first_x):
            #np.savetxt(dir1_path+'/hateb_data_'+sub_name+lt+'.csv',np.column_stack([self.first_x,self.first_y]),delimiter=',', header="first_x,first_y", comments="")
            #np.savetxt(dir2_path+'/hateb_data_'+sub_name+lt+'.csv',np.column_stack([self.actual_position_x,self.actual_position_y]),delimiter=',', header="actual_x,actual_y", comments="")
            #np.savetxt(dir3_path+'/hateb_data_'+sub_name+lt+'.csv',np.column_stack([self.ttg,self.ttg_error]),delimiter=',', header="ttg,ttg_error_prev", comments="")
            #np.savetxt(dir4_path+'/hateb_data_'+sub_name+lt+'.csv',np.column_stack([self.min_hum_dist,self.total_time]),delimiter=',', header="min_human_dist,total_travel_time", comments="")
            print('Saved data files')
            print('Cost of TTC = ',self.cost_ttc)
            print(self.ttc_arr)
            # print(self.first_x)



if __name__ == "__main__":
    compare = CompareTrajs()

    # For running just ros
    rospy.spin()

    ## For drawing
    #while(1):
    #    now = rospy.Time.now()
    #    if (now-compare.last_time).secs > 1.0:
    #        compare.first_path.poses = []
    #        compare.done = False

    #    compare.ttg_draw()
    #    plt.show()
    #    time.sleep(0.1)
