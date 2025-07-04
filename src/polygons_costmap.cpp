#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

// costmap converter libraries
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/costmap_converter_interface.h>


int main(int argc, char** argv) {
    // Inizializza il nodo ROS
    ros::init(argc, argv, "polygons_costmap");
    
    // Crea un handle del nodo
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    costmap_2d::Costmap2DROS costmap_ros("", tfBuffer);

    struct CostmapConverterPlugin
    {
        std::string costmap_converter_plugin;
        double costmap_converter_rate      = 5;
        bool costmap_converter_spin_thread = true;
        
    } _costmap_conv_params;

    costmap_2d::Costmap2D* _costmap;

    // The costmap converter plugin setting
    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> _costmap_converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons");  // Load costmap converter plugins at runtime
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> _costmap_converter;              // Store the current costmap_converter
    costmap_converter::ObstacleArrayMsg _custom_obstacle_msg;  // Copy of the most recent obstacle message

    nh.param("costmap_converter_plugin", _costmap_conv_params.costmap_converter_plugin, _costmap_conv_params.costmap_converter_plugin);
    nh.param("costmap_converter_rate",   _costmap_conv_params.costmap_converter_rate,   _costmap_conv_params.costmap_converter_rate);
    nh.param("costmap_converter_spin_thread", _costmap_conv_params.costmap_converter_spin_thread, _costmap_conv_params.costmap_converter_spin_thread);

    _costmap     = costmap_ros.getCostmap();

    if (!_costmap_conv_params.costmap_converter_plugin.empty()){
            try{
                _costmap_converter         = _costmap_converter_loader.createInstance(_costmap_conv_params.costmap_converter_plugin);
                std::string converter_name = _costmap_converter_loader.getName(_costmap_conv_params.costmap_converter_plugin);
                // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                boost::replace_all(converter_name, "::", "/");
                _costmap_converter->setOdomTopic("/odom"); 
                _costmap_converter->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
                _costmap_converter->setCostmap2D(_costmap);
                _costmap_converter->startWorker(ros::Rate(_costmap_conv_params.costmap_converter_rate), _costmap, _costmap_conv_params.costmap_converter_spin_thread);
                
                ROS_INFO_STREAM("Costmap conversion plugin " << _costmap_conv_params.costmap_converter_plugin << " loaded.");

            }catch (pluginlib::PluginlibException& ex){
                ROS_WARN(
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error "
                    "message: %s", ex.what());
                _costmap_converter.reset();
            }
        }else
            ROS_INFO("No costmap conversion plugin specified.");

    ROS_INFO("Polygons Costmap inizializzato");



    ros::Publisher obstacle_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>(
        "/polygons_costmap/obstacles", 1);

    while (ros::ok()) {
        costmap_converter::ObstacleArrayMsg obstacles;

        // 4. Update from costmap
        _costmap_converter->compute();
        _costmap_converter->getObstacles(obstacles);

        obstacle_pub.publish(obstacles);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}


double CostmapConverter::getDisstancePoints(geometry_msgs::Point p1, geometry_msgs::Point p2 )
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void CostmapConverter::updateObstacleContainerWithCostmapConverter()
{
    if (!_costmap_converter) return;

    // Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = _costmap_converter->getObstacles();
    if (!obstacles) return;


    std::vector<geometry_msgs::Pose> points = {};
    points.resize( obstacles->obstacles.size() );

    int index = 0;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
        const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
        const geometry_msgs::Polygon* polygon          = &obstacle->polygon;

        geometry_msgs::Pose errorpoint = geometry_msgs::Pose();
        errorpoint.position.x = polygon->points[0].x;
        errorpoint.position.y = polygon->points[0].y;

        // TODO: put the 2 to be a configuration parameter
        if(CostmapConverter::getDisstancePoints(_current_position, errorpoint.position) < 3 )
        {
            points[index] = errorpoint;
            index++;
        }
    }

    points.resize( index );


    geometry_msgs::PoseArray pa = geometry_msgs::PoseArray();
    pa.poses = points;
    pa.header.frame_id = "odom";
    
    _pub.publish(pa);

    
    
}

