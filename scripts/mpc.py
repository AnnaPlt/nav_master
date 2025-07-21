#!/usr/bin/env python3
import rospy
import numpy as np
import nav_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import multiprocessing as mp
import std_msgs.msg
import mpc_util
import time
import tf2_ros, tf2_geometry_msgs
from nav_master.srv import endPoint, endPointResponse
from dynamic_reconfigure.server import Server
#from nav_master.config import mpcParamsConfig

class ModelPredictiveControl:
    def __init__(self) :
        rospy.init_node("model_predictive_control")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.dynamic_callback()
        self.mpc_util = mpc_util.MPCUtil()
        self.model = self.mpc_util.get_model(self.Ts)
        
        self.new_goal = False
        self.goal = [2.0, 0.0, 0.0, 0.0, 0.0]
        self.odom = []
        self.obstacles = []

        # Subscribers
        self.odometry_sub = rospy.Subscriber("/odometry/filtered", nav_msgs.msg.Odometry, self.onReceiveOdometry)
        service = rospy.Service("/end_point_service", endPoint, self.onReceiveGoal)
        self.obstacles_sub = rospy.Subscriber("/scan_polar_map", sensor_msgs.msg.LaserScan, self.onReceiveObstacles)
        self.goal_pub = rospy.Publisher("/goal", geometry_msgs.msg.PoseStamped, queue_size=1)
        #srv_config = Server(mpcParamsConfig, self.dynamic_callback)

        # Publishers
        self.plan_pub = rospy.Publisher("/mpc_plan", nav_msgs.msg.Path, queue_size=1)
        self.pub_result = rospy.Publisher("/goal_reached", std_msgs.msg.Bool, queue_size=1)

    def dynamic_callback(self):
        self.time_between_controllers = rospy.get_param("~time_between_controllers", 1.0) # [s] 
        self.object_point_survive_time = rospy.get_param("~object_point_survive_time", 15.0)   # [s]
        self.max_num_obstacles = rospy.get_param("~max_num_obstacles", 50)
        self.command_future_view = rospy.get_param("~command_future_view", 12)
        self.Ts = rospy.get_param("~Ts", 0.1)
        self.distance_to_goal = rospy.get_param("~distance_to_goal", 0.5)
        self.frame_id_mpc = rospy.get_param("~frame_id_mpc", "odom")

    def onReceiveOdometry(self, msg):
        # Process odometry data
    
        self.odom = self.mpc_util.convert_pose_data_to_state(msg.pose)
        self.odom[3] = msg.twist.twist.linear.x 
        self.odom[4] = msg.twist.twist.angular.z 

        return
    

    def onReceiveGoal(self, req):
        end_point = req.end_point
        if(end_point.header.frame_id != self.frame_id_mpc):
            rospy.loginfo("frame id of end point is not {}".format(self.frame_id_mpc))
            return endPointResponse(success=False)

        new_goal = self.mpc_util.convert_pose_data_to_state(end_point)
        if(np.sqrt((new_goal[0]-self.goal[0])**2 + (new_goal[1]-self.goal[1])**2) < self.distance_to_goal):
            return endPointResponse(success=False)
        
        self.goal = new_goal
        self.goal_pub.publish(end_point)
        self.new_goal = True
        rospy.loginfo("New goal received: {}".format(self.goal))
        
        return endPointResponse(success=True)
    
    
    def onReceiveObstacles(self, msg):
        laser_ranges = np.array(msg.ranges)
        obstacles_list = []
        current_time = time.time()

        for indx, laser in enumerate(laser_ranges):
            #se serve ridurre il numero di indici da considerare
            if np.isinf(laser) or np.isnan(laser):
                continue
            
            # Convert polar coordinates to Cartesian coordinates
            angle = indx * np.pi / 180.0
            x = laser * np.cos(angle)
            y = laser * np.sin(angle)



            #convert to odom frame id
            transform = self.tf_buffer.lookup_transform(self.frame_id_mpc, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            obj_in_odom = tf2_geometry_msgs.do_transform_point(geometry_msgs.msg.PointStamped(header=msg.header, point=geometry_msgs.msg.Point(x=x, y=y, z=0)), transform)

            new_obj = [obj_in_odom.point.x, obj_in_odom.point.y, current_time]
            obstacles_list.append(new_obj)

        self.obstacles = obstacles_list #contains at most 360 values, 4m of range max

        return

                

    def remove_old_obstacles(self):
        if(not self.obstacles):
            return    
        new_set = []
        current_time = time.time()
        for obj in self.obstacles:
            if ( ( current_time - obj[2] ) < self.object_point_survive_time ):
                new_set.append( obj )
        self.obstacles = new_set

        return


    def run(self):
        rate = rospy.Rate(1/self.Ts) 
        last_mpc = time.time()-2

        while not rospy.is_shutdown():
            
            if(len(self.odom)<1 or len(self.obstacles)<1):
                rospy.logwarn("Waiting for odometry and obstacles data...")
                rospy.sleep(1.0)
                continue


            if(self.new_goal):
                self.remove_old_obstacles()
                if(time.time() - last_mpc > self.time_between_controllers):
                    mpc = self.mpc_util.get_controller(self.goal, self.Ts, self.model, self.obstacles, self.odom)
                    last_mpc = time.time()
                    #rospy.loginfo("MPC controller created")
            
                if(np.linalg.norm(self.odom[:2]-self.goal[:2])> self.distance_to_goal):
                    #rospy.loginfo("Running...")
                    mpc.make_step(self.odom)
                    self.plan_pub.publish(self.mpc_util.get_local_plan(mpc.data, self.frame_id_mpc))
                else:
                    rospy.loginfo("Goal reached, stopping MPC.")
                    self.pub_result.publish(std_msgs.msg.Bool(data=True))
                    self.new_goal = False

            rate.sleep()

        return



def main():
    
    mp.set_start_method('spawn')
    mpc = ModelPredictiveControl()
    mpc.run()



if __name__ == '__main__':
    main()
