#ifndef SET_END_POINT_H
#define SET_END_POINT_H


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_master/endPoint.h>

namespace set_end_point{
    class setEndPoint{
        public:
            setEndPoint(ros::NodeHandle& nh);
            ~setEndPoint();
            void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
            void sendEndPointRequest();
            void update();
            void run();
            bool send_request();

        private:
            ros::ServiceClient client_endPoint; 
            ros::Subscriber joy_sub_;
            ros::Publisher end_point_pub_;

            nav_master::endPoint srv;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener; 
            double v;
            double s;
            double raggio;
            int prev_msg;
            geometry_msgs::PoseStamped end_point_current;        
            geometry_msgs::PoseStamped new_end_point_odom;
    };

};
#endif