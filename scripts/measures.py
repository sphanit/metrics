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

HUMAN_ROBOT_TTC=12
HUMAN_ROBOT_TTCplus=17
HUMAN_ROBOT_VISIBILITY=15
STATIC = 0
DYN = 1

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
        self.start = False
        self.finished = True
        self.min_hum_dist = -1.0
        self.total_time = -1.0
        self.cost_ttc = 0
        self.ttc_arr = []
        self.safety_arr = []
        self.visibility_arr = []
        self.ttc_arr_cost = []
        self.safety_arr_cost = []
        self.visibility_arr_cost = []
        self.robot_vel_x = []
        self.robot_vel_y = []
        self.robot_vel_theta = []

        self.current_pose = Pose()
        self.current_vel = Twist()

        self.human_pose = Pose()
        self.human_vel = Twist()

        self.human_theta = 0
        self.robot_theta = 0
        self.rel_V = [0,0]


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
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/optimization_costs",OptimizationCostArray,self.opCostCB,queue_size=1)


        self.ttg_pub = rospy.Publisher('ttg', Float64, queue_size=1)
        self.ttg_prev_pub = rospy.Publisher('ttg_diff_prev', Float64, queue_size=1)
        self.ttg_first_pub = rospy.Publisher('ttg_diff_start', Float64, queue_size=1)
        self.total_time_pub = rospy.Publisher('total_time', Float64, queue_size=1)

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
                    "HUMAN_ROBOT_TTCplus": [],
                    "HUMAN_ROBOT_VISIBILITY" : []
                }

        self.listener = tf.TransformListener()
        self.last_time = rospy.Time.now()
        self.begin_time = rospy.Time.now()
        self.dt = rospy.Time.now()
        self.r_dt = 0
        self.r_dt_miss = 0

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
        (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

        (r,p,y) = euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.actual_position_x.append(msg.position.x)
        self.actual_position_y.append(msg.position.y)
        self.actual_position_theta.append(y)
            # self.current_pose = msg
        # self.robot_theta = y

    def opCostCB(self,msg):
        # if(len(self.costs_sum.costs)==0):
        #     self.costs_sum = msg
        # else:
        #     for i in range(0,len(msg.costs)):
        #         self.costs_sum.costs[i].cost += msg.costs[i].cost
        #
        # self.opcosts.append(msg)
        self.start = True
        self.finished = False
        cost_value = 0.0
        costs_array = []
        # print("opCostcb")
        for cost in msg.costs:
            # print('costs=',cost)
            if(cost.type==HUMAN_ROBOT_TTC):
                cost_value = cost.cost
                costs_array = cost.costs_arr
                self.tmp_costs["HUMAN_ROBOT_TTC"].append([cost_value, costs_array])
                self.cal_ttc()
            if(cost.type==HUMAN_ROBOT_TTCplus):
                cost_value = cost.cost
                costs_array = cost.costs_arr
                print(cost.costs_arr)

                self.tmp_costs["HUMAN_ROBOT_TTCplus"].append([cost_value, costs_array])
                self.cal_ttcplus()
            if(cost.type==HUMAN_ROBOT_VISIBILITY):
                cost_value = cost.cost
                costs_array = cost.costs_arr
                self.tmp_costs["HUMAN_ROBOT_VISIBILITY"].append([cost_value, costs_array])
                self.cal_visibility()


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
                # self.ttc_arr = []
                # self.ttc_arr_cost = []
                self.first_path.poses = []
                self.done = False
                self.start= False
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
                    tp = segment.pose.pose.orientation
                    (r,p,y) = euler_from_quaternion([tp.x,tp.y,tp.z,tp.w])
                    self.human_theta = y


    def plot_traj(self,msg):
        # Use this when the robot pose is not being published
        # self.listener.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(0.05))
        # (trans,rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
        # (r,p,y) = euler_from_quaternion(rot)
        #
        # self.current_pose.position.x = trans[0]
        # self.current_pose.position.y = trans[1]
        # self.current_pose.position.z = trans[2]
        # self.current_pose.orientation.x = rot[0]
        # self.current_pose.orientation.y = rot[1]
        # self.current_pose.orientation.z = rot[2]
        # self.current_pose.orientation.w = rot[3]
        # self.robot_theta = y

        # plt.figure(1)
        # if(len(self.current_path.poses)!=0):
        #     plt.plot(self.current_path.poses[0].pose.position.x, self.current_path.poses[0].pose.position.y,'r*')
        #     plt.plot(self.actual_position_x, self.actual_position_y,'b')
        #     # plt.axis('square')
        #     plt.axis('equal')
        #     plt.title('robot_path')
        #
        # plt.figure(2)
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
        plt.figure(1)
        plt.clf()
        if msg == "HUMAN_ROBOT_TTCplus" or "HUMAN_ROBOT_TTC":
            plt.plot(self.ttc_arr_cost,'b')
        elif msg == "HUMAN_ROBOT_VISIBILITY":
            plt.plot(self.visibility_arr_cost,'b')
        # plt.axis('equal')
        plt.title('Calculated')

        plt.figure(2)
        plt.clf()
        idx = len(self.tmp_costs[msg])-1
        plt.plot(self.tmp_costs[msg][idx][1],'r')
        # plt.axis('equal')
        plt.title('Optim')

        plt.figure(3)
        plt.clf()
        r_x = self.current_pose.position.x
        r_y = self.current_pose.position.y
        h_x = self.human_pose.position.x
        h_y = self.human_pose.position.y

        circle1 = plt.Circle((h_x,h_y), 0.3, color='r')
        circle2 = plt.Circle((r_x,r_y), 0.34, color='y')
        circle3 = plt.Circle((h_x,h_y), 0.64, color='b', fill=False)
        xt, yt = self.get_circ_tangents([h_x,h_y], 0.64, [r_x,r_y])

        # plt.gca().add_artist(circle1)
        ax = plt.gca()
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        ax.add_patch(circle3)
        # ax.set_xlim((-2, 28))
        # ax.set_ylim((-10, 20))
        ax.set_xlim((0, 15))
        ax.set_ylim((0, 5))
        ax.set_aspect('equal')
        ax.plot()
        dir = [math.cos(self.human_theta)*0.3, math.sin(self.human_theta)*0.3]
        plt.arrow(h_x, h_y, dir[0], dir[1], width = 0.05)

        dir_r = [math.cos(self.robot_theta)*0.34, math.sin(self.robot_theta)*0.34]
        plt.arrow(r_x, r_y, dir_r[0], dir_r[1], width = 0.05)
        rn = (np.linalg.norm([self.rel_V[0],self.rel_V[1]]))
        plt.arrow(r_x, r_y, self.rel_V[0]/rn, self.rel_V[1]/rn, width = 0.05, ec='red', color='r')
        if xt is not None:
            plt.plot([r_x,xt[0]],[r_y,yt[0]],'k')
            plt.plot([r_x,xt[1]],[r_y,yt[1]],'k')

        # line = plt.plot([r_x,h_x],[r_y,h_y])[0]
        # self.add_arrow(line, direction = 'right')


        # ax.add_artist(circle1)
        # plt.pause(0.05)
        plt.pause(0.05)
        plt.draw()
        plt.show()

    def cal_ttc(self):
        # double dt = (current_time_ - last_time_).toSec();
        # double theta = odom_.z;
        # double costh = cos(theta);
        # double sinth = sin(theta);
        #
        # computeBaseVelocity();
        #
        # double odom_delta_x = (odom_vel_.linear.x * costh -
        #                           odom_vel_.linear.y * sinth) * dt;
        # double odom_delta_y = (odom_vel_.linear.x * sinth +
        #                           odom_vel_.linear.y * costh) * dt;
        # double odom_delta_th = odom_vel_.angular.z * dt;
        #
        # odom_.x += odom_delta_x;
        # odom_.y += odom_delta_y;
        # odom_.z += odom_delta_th;
        if self.start:
            robot_radius = 0.34
            human_radius = 0.3
            radius_sum_ = robot_radius + human_radius
            radius_sum_sq_ = radius_sum_ * radius_sum_

            # theta = self.current_vel.angular.z
            theta = self.robot_theta
            r_vel = np.array([self.current_vel.linear.x*np.cos(theta) - self.current_vel.linear.y*np.sin(theta),
                    self.current_vel.linear.x*np.sin(theta) + self.current_vel.linear.y*np.cos(theta)])

            theta_h = self.human_theta
            # h_vel = np.array([self.human_vel.linear.x*np.cos(theta_h) - self.human_vel.linear.y*np.sin(theta_h),
                    # self.human_vel.linear.x*np.sin(theta_h) + self.human_vel.linear.y*np.cos(theta_h)])
            h_vel = np.array([self.human_vel.linear.x, self.human_vel.linear.y])
            C = np.array([self.human_pose.position.x, self.human_pose.position.y]) - np.array([self.current_pose.position.x, self.current_pose.position.y])
            ttc = np.inf
            C_sq = np.dot(C,C)
            # print(C_sq)
            # print(C)

            if C_sq <= radius_sum_sq_:
                ttc = 0.0
            else:
                V = r_vel - h_vel
                print(h_vel)
                # print(r_vel)
                self.rel_V = V

                # print(V)
                C_dot_V = np.dot(C,V)
                if C_dot_V > 0:
                    V_sq = np.dot(V,V)
                    f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_))
                    if f > 0:
                        ttc = (C_dot_V - math.sqrt(f)) / V_sq

            # print(ttc)

            if ttc < np.inf:
                self.ttc_arr.append(ttc)
                tp = self.penaltyBoundFromBelow(ttc, 10.0, 0.01)/C_sq
                self.ttc_arr_cost.append(tp*tp)

            else:
                self.ttc_arr_cost.append(0.0)
                self.ttc_arr.append(-1.0)

    def cal_ttcplus(self):
        if self.start:
            robot_radius = 0.34
            human_radius = 0.3
            radius_sum_ = robot_radius + human_radius
            radius_sum_sq_ = radius_sum_ * radius_sum_

            theta = self.robot_theta
            r_vel = np.array([self.current_vel.linear.x*np.cos(theta) - self.current_vel.linear.y*np.sin(theta),
                    self.current_vel.linear.x*np.sin(theta) + self.current_vel.linear.y*np.cos(theta)])

            theta_h = self.human_theta
            # h_vel = np.array([self.human_vel.linear.x*np.cos(theta_h) - self.human_vel.linear.y*np.sin(theta_h),
                    # self.human_vel.linear.x*np.sin(theta_h) + self.human_vel.linear.y*np.cos(theta_h)])
            h_vel = np.array([self.human_vel.linear.x, self.human_vel.linear.y])

            C = np.array([self.human_pose.position.x, self.human_pose.position.y]) - np.array([self.current_pose.position.x, self.current_pose.position.y])
            ttc = np.inf
            C_sq = np.dot(C,C)
            # print(C_sq)
            # print(C)

            if C_sq <= radius_sum_sq_:
                ttc = 0.0
            else:
                V = r_vel - h_vel
                self.rel_V = V
                # print(V)
                C_dot_V = np.dot(C,V)
                if C_dot_V > 0:
                    V_sq = np.dot(V,V)
                    f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_))
                    if f > 0:
                        ttc = (C_dot_V - math.sqrt(f)) / V_sq

            # print(ttc)

            if ttc < np.inf:
                now = rospy.Time.now()
                self.r_dt = self.r_dt + (now-self.dt).to_sec()
                self.r_dt_miss = 0.
                self.dt = now

                if self.r_dt >= 1.0:
                    self.ttc_arr.append(ttc)
                    # tp = self.penaltyBoundFromBelow(ttc, 10.0, 0.01)/C_sq
                    tp = self.penaltyBoundFromBelow(ttc, 10.0, 0.01)
                    self.ttc_arr_cost.append(tp*tp)
                else:
                    self.ttc_arr.append(-1.0)
                    self.ttc_arr_cost.append(0.0)

            else:
                now = rospy.Time.now()
                self.r_dt_miss = self.r_dt_miss + (now-self.dt).to_sec()
                self.dt = now
                if self.r_dt_miss >= 5.0:
                    self.r_dt = 0.0
                    self.r_dt_miss =0.0

                self.ttc_arr_cost.append(0.0)
                self.ttc_arr.append(-1.0)

    def cal_safety(self):
        robot_radius = 0.34
        human_radius = 0.3
        radius_sum_ = robot_radius + human_radius

        hr_dist = np.linalg.norm([self.current_pose.position.x-self.human_pose.position.x,self.current_pose.position.y-self.human_pose.position.y])
        hr_dist = hr_dist - radius_sum_
        tp = self.penaltyBoundFromBelow(hr_dist, 0.5, 0.01)
        self.safety_arr.append(tp)
        self.safety_arr_cost.append(tp*tp)

    def cal_visibility(self):
        d_rtoh = np.array([self.human_pose.position.x, self.human_pose.position.y]) - np.array([self.current_pose.position.x, self.current_pose.position.y])
        d_htor = np.array([self.current_pose.position.x, self.current_pose.position.y]) - np.array([self.human_pose.position.x, self.human_pose.position.y])
        humanLookAt = np.array([math.cos(self.human_theta), math.sin(self.human_theta)])

        deltaPsi = abs(math.acos(np.dot(humanLookAt,d_htor)/(np.linalg.norm(humanLookAt)*np.linalg.norm(d_htor))))
        if deltaPsi >= np.pi/3:
            c_visibility = 5*((2*math.pow(2,-(math.pow(d_rtoh[0],2)))) + (2*math.pow(2,-(math.pow(d_rtoh[1],2)))))
        else:
            c_visibility = 0.0

        self.visibility_arr.append(c_visibility)

        tp = self.penaltyBoundFromAbove(c_visibility, 1.5, 0.01)
        self.visibility_arr_cost.append(tp*tp)

    def penaltyBoundFromBelow(self, var, a, eps):
        if  var >= a+eps:
            return 0.0
        else:
            return (-var + (a+eps))

    def penaltyBoundFromAbove(self, var, a, eps):
        if  var <= a-eps:
            return 0.0
        else:
            return (var - (a-eps))

    def add_arrow(self, line, position=None, direction='right', size=15, color=None):
        """
        add an arrow to a line.

        line:       Line2D object
        position:   x-position of the arrow. If None, mean of xdata is taken
        direction:  'left' or 'right'
        size:       size of the arrow in fontsize points
        color:      if None, line color is taken.
        """
        if color is None:
            color = line.get_color()

        xdata = line.get_xdata()
        ydata = line.get_ydata()

        if position is None:
            position = xdata.mean()
        # find closest index
        #start_ind = np.argmin(np.absolute(xdata - position))
        if direction == 'right':
            start_ind = len(xdata)-2
            end_ind = start_ind + 1
        else:
            start_ind = len(xdata)-1
            end_ind = start_ind - 1

        line.axes.annotate('',
            xytext=(xdata[start_ind], ydata[start_ind]),
            xy=(xdata[end_ind], ydata[end_ind]),
            arrowprops=dict(arrowstyle="->", color=color),
            size=size
        )

    def get_circ_tangents(self, ctr, r, XY):
        a = np.linspace(0, 2*np.pi, num=200);
        x = ctr[0] + r*np.cos(a)
        y = ctr[1] + r*np.sin(a)
        c = math.hypot(XY[0]-ctr[0],XY[1]-ctr[1])
        if c <= r:
            print('Tangents not possible from this point')
            return None, None
        b = math.sqrt(c**2 - r**2)                                             # See Wikipedia Reference
        alfa = math.acos((r**2 - b**2 - c**2)/(-2*b*c))                        # See Wikipedia Reference
        beta = math.acos((b**2 - r**2 - c**2)/(-2*r*c))                        # See Wikipedia Reference
        pt_ctr_angl = math.atan2(-(XY[1]-ctr[1]),-(XY[0]-ctr[0]))               # Angle From Point To Circle Centre
        alfac = [pt_ctr_angl + alfa, pt_ctr_angl - alfa]                   # Angles From Point For Tangents
        xtng = [XY[0] + b*math.cos(alfac[0]), XY[0] + b*math.cos(alfac[1])]                # Tangent Point x Coordinates
        ytng = [XY[1] + b*math.sin(alfac[0]), XY[1] + b*math.sin(alfac[1])]                # Tangent Point y Coordinates

        return xtng, ytng


    def run_plotter(self, msg, mode):
        if mode == STATIC and not self.finished:
            self.plot_traj(msg)
            self.finished = True
        elif mode == DYN and not self.finished:
            self.plot_traj(msg)


if __name__ == "__main__":
    plotter = PlotterHateb()
    cost = "HUMAN_ROBOT_TTCplus"
    mode = DYN
    while True:
        try:
            plotter.run_plotter(cost, mode)
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
