#include "../include/nav_master/navigationStack.h"



namespace navigation_stack{

NavigationStack::NavigationStack(ros::NodeHandle nh) : nh_(nh), tfListener(tf_), new_costmap(false){
    
    ns_active = false;
    new_plan = false;

    default_config.__getDefault__();

    updateParams(default_config);

    init_VirtualSensors();
    init_Communication();

    f_reconfigure = boost::bind(&NavigationStack::configCallback, this, _1, _2);
    reconfigure_server_.setCallback(f_reconfigure);

    ROS_INFO("Press A button to start the navigation stack");
}

NavigationStack::~NavigationStack(){
    ROS_INFO(" Stopping navigation stack");
    util_sendStopMsg();
    ros::shutdown();
}


void NavigationStack::updateParams(artificial_potential_fields::apfParamsConfig &config) {
    delta = config.delta_polarmap;
    cost_obstacle = config.obstacle_cost_threshold;
    VS_frame_id = config.virtual_sensor_reference_frame;
    max_radius = config.max_radius;
    sampling_distance = config.sampling_distance;
    max_angle = config.max_angle;
    linear_scaling = config.scaling_factor_linear_velocity;
    angular_scaling = config.scaling_factor_angular_velocity;
    strength_attractors_angular_velocity = config.strength_attractors_angular_velocity;
    //setMaxVelocities();
}

void NavigationStack::configCallback(artificial_potential_fields::apfParamsConfig &config, uint32_t level) {
    ROS_INFO("updating params");
    updateParams(config); 
    vs->setParams(config); // da cambiare se passiamo più vs
}


void NavigationStack::init_VirtualSensors(){
    vs = new VirtualSensor(VS_frame_id);
}

void NavigationStack::init_Communication(){
    costmap_sub_ = nh_.subscribe("/local_costmap/costmap", 1, &NavigationStack::onReceiveCostmap, this);
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/nav_stack/cmd_vel", 1);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_polar_map", 1);
    joy_sub_ = nh_.subscribe("/joy", 1, &NavigationStack::onReceiveJoy, this);
    path_sub_ = nh_.subscribe("/mpc_plan", 1, &NavigationStack::onReceivePath, this); //non esiste questo topic al momento
    plan_pub_ = nh_.advertise<nav_msgs::Path>("/sampled_plan", 1);
    nav_result_sub_ = nh_.subscribe("/goal_reached", 1, &NavigationStack::onReceiveNavResult, this); // per ora lo uso per il toggle
    img_pub = nh_.advertise<sensor_msgs::Image>("/apf", 1);
    matrix_pub = nh_.advertise<std_msgs::Float64MultiArray>("/matrix", 1);

}


void NavigationStack::setMaxVelocities(){

    const double delta_alpha = M_PI / 180.0;
    /*double result_angular = 0.0;

    for (double alpha = -M_PI; alpha < M_PI; alpha += delta_alpha) {
        double f = vs->ap_field.kernel(alpha);
        result_angular += std::abs(f);
    }
    maxAngularVelocity = result_angular;
    //ROS_INFO("max Angular Velocity set to: %f", maxAngularVelocity);*/

    double min_radius = 1.2;
    double result_linear = 0.0;
    for (double alpha = -max_angle; alpha < max_angle; alpha += delta_alpha){
        double l = std::max(0.0, (max_angle - std::abs(alpha))/max_angle);
        result_linear += l;
    }
    maxLinearVelocityRepulsors = result_linear*1/min_radius;
    //ROS_INFO("max Linear Velocity Repulsors set to: %f", maxLinearVelocityRepulsors);
}


void NavigationStack::onReceiveNavResult(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data){
        ROS_INFO("Goal reached, stopping navigation stack");
        util_sendStopMsg();
        new_plan = false;
    }
}


void NavigationStack::onReceivePath(const nav_msgs::Path& msg) {
    
    sampled_plan = path_sampler.getSampledPlan(sampling_distance, msg);
    
    //ROS_INFO("Received new path with %lu poses", sampled_plan.poses.size());
    sampled_plan.header.frame_id = msg.header.frame_id;
    plan_pub_.publish(sampled_plan);
    goal_pose = sampled_plan.poses.back(); // l'ultimo goal è l'ultimo della path
    //ROS_INFO("Goal pose set to: [%f, %f]", goal_pose.pose.position.x, goal_pose.pose.position.y);
    //ts = tf_.lookupTransform("odom", msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
    //tf2::doTransform(goal_pose, goal_pose, ts); // trasformo goal in odom frame
    //uso mpc per ottenere un path e lo metto in mpc_plan
    new_plan = true;
}

void NavigationStack::onReceiveJoy(const sensor_msgs::Joy::ConstPtr& msg) {

    if(msg->buttons[1] == 1){ //button A
        ns_active = !ns_active; // toggle state
        if(ns_active){
            ROS_INFO("Navigation stack started");
        } else {
            util_sendStopMsg();
            ROS_INFO("Navigation stack stopped");
        }
    }

}

void NavigationStack::onReceiveCostmap(nav_msgs::OccupancyGrid msg) {
    
    costmap_.occupancy_data = msg.data;
    costmap_.frame_id    = msg.header.frame_id;
    costmap_.origin = msg.info.origin;
    costmap_.resolution = msg.info.resolution;
    costmap_.width = msg.info.width;
    costmap_.height = msg.info.height;

    new_costmap = true;
 
}

void NavigationStack::buildPolarMap(){

    polar_map.clear();
    polar_map.resize(360, std::numeric_limits<double>::infinity()); //std::nan()
    ts = tf_.lookupTransform(costmap_.frame_id, VS_frame_id, ros::Time(0), ros::Duration(1.0));
    double yaw =  tf::getYaw(ts.transform.rotation);
    if(std::abs(yaw)<0.0175) yaw = 0.0;

    for(int theta = 0; theta < 360; theta += delta){
        for(double r = 0.0; r < max_radius; r+=0.05){

            double x = r*cos(theta*M_PI/180.0 + yaw );
            double y = r*sin(theta*M_PI/180.0 + yaw );
            
            int x_cell = x/costmap_.resolution + costmap_.width/2;
            int y_cell = y/costmap_.resolution + costmap_.height/2;
           
            if(0 < x_cell && x_cell < costmap_.width && 0 < y_cell && y_cell < costmap_.height){
                int indx = y_cell * costmap_.width + x_cell;

                if(costmap_.occupancy_data[indx] > cost_obstacle){
                    polar_map[theta] = r; //rispetto footprint
                    break;
                }
            }
        }
    }

}


sensor_msgs::ImagePtr NavigationStack::eigenMatrixToImageMsg(const Eigen::MatrixXd& mat) {
    // Normalizza valori in [0,255]
    double min = mat.minCoeff();
    double max = mat.maxCoeff();
    Eigen::MatrixXd norm =  (mat.array() - min)*255;  // evita divisione per zero
    
    // Converti in cv::Mat mono 8 bit
    cv::Mat gray(mat.rows(), mat.cols(), CV_8UC1);
    for(int r = 0; r < mat.rows(); ++r) {
        for(int c = 0; c < mat.cols(); ++c) {
            gray.at<uint8_t>(r, c) = static_cast<uint8_t>(norm(r, c));
        }
    }
    //cv::flip(gray, gray, 0);
    // Applica colormap per renderla colorata
    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_JET); // puoi cambiare la mappa: COLORMAP_HOT, COLORMAP_PARULA, etc.

    // Crea messaggio ROS (bgr8 perché OpenCV usa BGR di default)
    return cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
}



void NavigationStack::util_sendLaserMsg(){
    //build laser msgs
    sensor_msgs::LaserScan laser_msg;
    laser_msg.header.frame_id = VS_frame_id;
    laser_msg.header.stamp = ros::Time::now();
    laser_msg.angle_min = 0.0;
    laser_msg.angle_max = 2*M_PI;
    laser_msg.angle_increment = (M_PI/180.0);
    laser_msg.range_min = 0.0;
    laser_msg.range_max = INFINITY;
    laser_msg.ranges = polar_map;
    scan_pub_.publish(laser_msg);
    
}

void NavigationStack::util_sendStopMsg(){
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    velocity_pub_.publish(stop_msg);
}


double NavigationStack::normalizeLinearVelocities(double lin_vel_rep, double lin_vel_attr) {
    //ROS_INFO("linear velocity repellor normalized: %f", (2*((lin_vel_rep-(-maxLinearVelocityRepulsors))/(2*maxLinearVelocityRepulsors))-1.0) );
    //ROS_INFO("linear velocity attractor normalized: %f", (2*((lin_vel_attr-(-max_radius))/(2*max_radius))-1.0));
    double linear_velocity = (2*((lin_vel_rep-(-maxLinearVelocityRepulsors))/(2*maxLinearVelocityRepulsors))-1.0) 
                + (2*((lin_vel_attr-(-max_radius))/(2*max_radius))-1.0);

    return linear_velocity;
}

void NavigationStack::run(){

    ros::Rate r(2);
    ros::Time last_map = ros::Time::now();
    ros::Time last_goal = ros::Time::now();
    while(ros::ok()){
        if (new_costmap) {
            last_map = ros::Time::now();

            buildPolarMap();
            util_sendLaserMsg();

            double lin_vel = 0.0;
            double ang_vel = 0.0;

            
            vs->emptySet();
            vs->setRepellors(polar_map);
            Eigen::MatrixXd omegaAll = vs->computeOmegaAll();
            std_msgs::Float64MultiArray msg;

            // riempi i dati: Eigen::MatrixXd è row-major o column-major, ma per semplicità:
            for(int r = 0; r < omegaAll.rows(); ++r) {
                for(int c = 0; c < omegaAll.cols(); ++c) {
                    msg.data.push_back(omegaAll(r, c));
                }
            }

            // opzionalmente: msg.layout.dim per indicare righe/colonne
            msg.layout.dim.resize(2);
            msg.layout.dim[0].label = "rows";
            msg.layout.dim[0].size = omegaAll.rows();
            msg.layout.dim[0].stride = omegaAll.rows() * omegaAll.cols();
            msg.layout.dim[1].label = "cols";
            msg.layout.dim[1].size = omegaAll.cols();
            msg.layout.dim[1].stride = omegaAll.cols();

            matrix_pub.publish(msg);
            //omegaAll = 2*(omegaAll.array() - omegaAll.minCoeff()) / (omegaAll.maxCoeff() - omegaAll.minCoeff())-1;
            std::ofstream file("matrice.csv");
            if (file.is_open()) {
                for (int i = 0; i < omegaAll.rows(); ++i) {
                    for (int j = 0; j < omegaAll.cols(); ++j) {
                        file << omegaAll(i, j);
                        if (j != omegaAll.cols() - 1)
                            file << ",";  // separatore
                    }
                    file << "\n";
                }
                file.close();
            } else {
                std::cerr << "Impossibile aprire il file!" << std::endl;
            }
            ang_vel = vs->getAngularVelocity(omegaAll, 1.0);

            lin_vel = vs->getLinearVelocity();
            ROS_INFO("ang_vel 1: %f", ang_vel);
            //ROS_INFO("lin_vel 1: %f", lin_vel);

            geometry_msgs::PoseStamped goal_in_fp;
            /*if(new_plan){
                ts = tf_.lookupTransform("base_footprint", goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
                tf2::doTransform(goal_pose, goal_in_fp, ts); // trasformo goal in odom frame
                
                vs->emptySet();
                vs->setAttractors(goal_in_fp);
                double ang_vel_ = vs->getAngularVelocity(strength_attractors_angular_velocity);
                ROS_INFO("ang_vel 2: %f", ang_vel_);
                ang_vel += ang_vel_;
            }*/
            
            if(ns_active){

                geometry_msgs::Twist cmd_vel;
                if(new_plan){
                    //ROS_INFO("got new plan");
                    cmd_vel.linear.x = lin_vel*linear_scaling;
                    //cmd_vel.linear.x = 0.0;
                }
                else{
                    cmd_vel.linear.x = 0.0;
                }
                cmd_vel.angular.z = ang_vel*angular_scaling;//std::max(std::min(1.0, ang_vel*angular_scaling), -1.0);
                
                velocity_pub_.publish(cmd_vel);
            }
            new_costmap = false;
            

        }

        ros::spinOnce();
        r.sleep();

    }

}

}