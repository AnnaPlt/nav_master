#!/usr/bin/env python3

import rospy
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros, tf2_geometry_msgs


class pythonClass:
    def __init__(self):
        self.polar_map = None
        self.sub_polar_map = rospy.Subscriber("/scan_polar_map", sensor_msgs.msg.LaserScan, self.set_repellors)
        self.pub_omega = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_plan = rospy.Subscriber("/sampled_plan", Path, self.set_goal)
        self.sub_joy = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.set_joy)
        self.ap_field = []
        self.nsett = 5
        self.step = 90/self.nsett
        self.t_rad = np.pi/180.0
        self.sigma = 2*np.pi/3 
        self.min_distance = 0.35
        self.decay = 0.7
        self.kmax_points = []
        self.kmax = self.setKmax()
        self.delta = self.get_delta()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.new_goal = False
        self.goal = None
        self.ns_active = False


    def set_joy(self, msg):
        if msg.buttons[1] == 1:
            self.ns_active = not self.ns_active


    def set_repellors(self, msg):
        self.ap_field.clear()
        self.polar_map = msg.ranges

        #chiamata all'arrivo di polar map
        for i in np.arange(0, len(self.polar_map), self.step): 
            min_dist = np.inf
            index = 0

            for j in np.arange(0, self.step):
                d = self.polar_map[int(i+j)]
                if d <= min_dist:
                    if np.abs(self.kernel(i+j, self.sigma)) < np.abs(self.kernel(index, self.sigma)):
                        continue
                    min_dist = d
                    index = i+j
            
            if(min_dist < np.inf):
                angle = index * self.t_rad
                if angle >= np.pi:
                    angle = angle - 2*np.pi
                distance = min_dist
                sign = -1

                repellor = [angle, distance, sign]
                self.ap_field.append(repellor)
        

    def set_goal(self, msg):
        goal = msg.poses.back()
        transform = self.buffer.lookup_transform("base_footprint", goal.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        goal_in_fp = tf2_geometry_msgs.do_transform_pose(goal, transform)
        
        alpha = np.arctan2(goal_in_fp.pose.position.y, goal_in_fp.pose.position.x)
        distance = np.sqrt(goal_in_fp.pose.position.x**2 + goal_in_fp.pose.position.y**2)
        self.goal = [alpha, distance, 1]

        self.new_goal = True

    def get_delta(self):
        delta = self.kmax/np.abs(self.kernel(np.pi/4, self.sigma))
        return delta
    

    def setKmax(self):
        
        step = self.step*self.t_rad
        kMax = 0
        kPoints = []
        pt = None

        for i in np.arange(-np.pi/2, 0, step): #avanza di step gradi
            max_k = 0
            
            for j in np.arange(0, step, self.t_rad):    #avanza di 1 grado 
                k = np.abs(self.kernel(i+j, self.sigma))
                if k > max_k:
                    max_k = k
                    pt = i+j
            
            kPoints.append(pt)
            kMax+= max_k

        self.kmax_points.append(kPoints)
        kPoints = []
        for i in np.arange(np.pi/2, np.pi, step):
            
            max_k = 0
            for j in np.arange(0, step, self.t_rad):
                
                k = np.abs(self.kernel(i+j, self.sigma))
                if k > max_k:
                    max_k = k
                    pt = i+j

            kPoints.append(pt)    
            kMax += max_k

        self.kmax_points.append(kPoints)
        return kMax
    

    def kernel(self, a, s):
        return np.sin(a) * np.cos(a) * np.exp(- (a**2) / (2 * s**2))

    def lambda_func(self, d, dmin, decay):
        d = np.array(d)
        return np.where(d < dmin, 1, np.exp(- (d - dmin) / decay))
    

    def setVelocity(self):
        omega_rep = 0
        for obj in self.ap_field:
            v = obj[2]*self.lambda_func(obj[1], self.min_distance, self.decay)*self.kernel(obj[0], self.sigma)
            v = v/(2*self.nsett)
            omega_rep+=v

        omega_attr = 0
        if(self.new_goal):
            omega_attr = self.lambda_func(obj[1], self.min_distance, self.decay)*self.kernel(obj[0], self.sigma)
            omega_attr = omega_attr*self.delta

        omega = (omega_rep + omega_attr)*0.3
        return omega



    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.ns_active):
                if(len(self.ap_field)==0):
                    rospy.logwarn("Waiting for repellors data...")
                    rospy.sleep(1.0)
                    continue
                omega = self.setVelocity()
                msg = Twist()
                msg.angular.z = omega
                self.pub_omega.publish(msg)
                rate.sleep()
            else:
                msg = Twist()
                msg.angular.z = 0
                self.pub_omega.publish(msg)
                rate.sleep()

def main():
    rospy.init_node('nav_stack', anonymous=True)
    og = pythonClass()
    og.run()


if __name__ == "__main__":
    main()