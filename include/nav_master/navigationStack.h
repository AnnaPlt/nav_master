#ifndef NAVIGATION_STACK_H_
#define NAVIGATION_STACK_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include "artificial_potential_fields/virtualSensor.h"
#include "pathSampler.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include </mnt/data/anna/anna_ws/devel/include/artificial_potential_fields/apfParamsConfig.h>



using namespace virtual_sensor;
using namespace path_sampler;

namespace navigation_stack{
    class NavigationStack{
        public:
            NavigationStack(ros::NodeHandle nh);
            ~NavigationStack();
            void run();

        private:
            sensor_msgs::ImagePtr eigenMatrixToImageMsg(const Eigen::MatrixXd& mat);
            void onReceiveCostmap(nav_msgs::OccupancyGrid msg);
            void onReceiveJoy(const sensor_msgs::Joy::ConstPtr& msg);
            void onReceivePath(const nav_msgs::Path& msg);
            void onReceiveNavResult(const std_msgs::Bool::ConstPtr& msg);
            void buildPolarMap();

            ros::NodeHandle nh_;
            std::vector<float> polar_map;
            VirtualSensor* vs;
            nav_msgs::Path sampled_plan;
            PathSampler path_sampler;

            //communication
            ros::Subscriber costmap_sub_;
            ros::Publisher velocity_pub_;
            ros::Subscriber joy_sub_;
            ros::Subscriber path_sub_;
            ros::Publisher scan_pub_;
            ros::Publisher goal_pub_;
            ros::Publisher plan_pub_;
            ros::Subscriber nav_result_sub_;
            

            //costmap
            struct maps_ptrs {
                std::vector<int8_t> occupancy_data;
                geometry_msgs::Pose origin;
                std::string frame_id;
                double resolution;
                unsigned int width, height;
            } costmap_;
            bool new_costmap;


            //tf
            tf2_ros::Buffer tf_;
            tf2_ros::TransformListener tfListener; 
            geometry_msgs::TransformStamped ts;

            //reconfigure
            void configCallback(artificial_potential_fields::apfParamsConfig &config, uint32_t level);
            artificial_potential_fields::apfParamsConfig default_config;
            dynamic_reconfigure::Server<artificial_potential_fields::apfParamsConfig> reconfigure_server_;
            dynamic_reconfigure::Server<artificial_potential_fields::apfParamsConfig>::CallbackType f_reconfigure;

            //reconfigurable params and derivatives
            int delta;
            int cost_obstacle;
            double sigma;
            double maxAngularVelocity;
            double maxLinearVelocityRepulsors;
            double max_radius;
            double linear_scaling;
            double angular_scaling;
            double sampling_distance;
            double max_angle;
            double min_distance;
            double rod;
            double strength_attractors_angular_velocity;
            std::string VS_frame_id;

            
            //utility and init
            void updateParams(artificial_potential_fields::apfParamsConfig &config);
            void util_sendLaserMsg();
            void init_VirtualSensors();
            void init_Communication();
            void util_sendStopMsg();
            void setMaxVelocities();
            double normalizeLinearVelocities(double lin_vel_rep, double lin_vel_attr);
            bool ns_active;
            bool new_plan;
            ros::Publisher img_pub;

            geometry_msgs::PoseStamped goal_pose;
            
    };
};

#endif