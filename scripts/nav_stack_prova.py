import rospy
import sensor_msgs.msg
import numpy as np
from geometry_msgs.msg import Twist


class pythonClass:
    def __init__(self):
        self.polar_map = None
        self.sub_polar_map = rospy.Subscriber("/scan_polar_map", sensor_msgs.msg.LaserScan, self.set_repellors)
        self.pub_omega = rospy.Publisher("/cmd_vel_py", Twist, queue_size=1)
        self.ap_field = []
        self.nsett = 20
        self.step = 360/self.nsett
        self.t_rad = np.pi/180.0
        self.sigma = 1.0
        self.min_distance = 0.5
        self.decay = 0.7
        self.kmax = self.setKmax()

    def set_repellors(self, msg):
        self.polar_map = msg

        #chiamata all'arrivo di polar map
        for i in range(0, len(self.polar_map.ranges), self.step): 
            min_dist = np.inf
            index = 0
            for j in range(self.step):
                d = self.polar_map.ranges[i+j]
                if d < min_dist:
                    min_dist = d
                    index = j

            if(min_dist != np.inf):
                angle = i * self.t_rad
                if angle >= np.pi:
                    angle = angle - 2*np.pi
                distance = self.polar_map.ranges[i+index]
                sign = -1

                repellor = [angle, distance, sign]
                self.ap_field.append(repellor)
        


    def setKmax(self):
        
        step = (360/20)*self.t_rad
        kMax = 0
        for i in np.arange(0, np.pi/2, step): 
            max_k = 0
            for j in np.arange(0, step, self.t_rad):    
                k = np.abs(self.kernel(i+j, self.sigma))
                if k > max_k:
                    max_k = k
            
                
            kMax+= max_k

        for i in np.arange(-np.pi, -np.pi/2, step):
            
            max_k = 0
            for j in np.arange(0, step, self.t_rad):
                
                k = np.abs(self.kernel(i+j, self.sigma))
                if k > max_k:
                    max_k = k
                
            kMax += max_k

        return kMax

    def kernel(self, a, s):
        return np.sin(a) * np.cos(a) * np.exp(- (a**2) / (2 * s**2))

    def lambda_func(self, d, dmin, decay):
        d = np.array(d)
        return np.where(d < dmin, 1, np.exp(- (d - dmin) / decay))
    

    def setVelocity(self):
        omega = 0
        for obj in self.ap_field:
            v = obj[2]*self.lambda_func(obj[1], self.min_distance, self.decay)*self.kernel(obj[0], self.sigma)
            v = v * self.nsett/(2*self.kmax)
            omega+=v

        return omega



    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(len(self.ap_field)<1):
                rospy.logwarn("Waiting for repellors data...")
                rospy.sleep(1.0)
                continue
            omega = self.setVelocity()
            msg = Twist()
            msg.angular.z = omega
            self.pub_omega.publish(msg)
            rate.sleep()

def main():
    rospy.init_node('nav_stack', anonymous=True)
    og = pythonClass()
    og.run()


if __name__ == "__main__":
    main()