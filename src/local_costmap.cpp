#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

int main(int argc, char** argv) {
    // Inizializza il nodo ROS
    ros::init(argc, argv, "local_costmap");
    
    // Crea un handle del nodo
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    costmap_2d::Costmap2DROS costmap("", tfBuffer);
    ROS_INFO("Costmap inizializzato");

    ros::spin();
    
    return 0;
}