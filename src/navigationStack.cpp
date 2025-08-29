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



void NavigationStack::onReceiveNavResult(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data){
        ROS_INFO("Goal reached, stopping navigation stack");
        util_sendStopMsg();
        new_plan = false;
    }
}


void NavigationStack::onReceivePath(const nav_msgs::Path::ConstPtr& msg) {
    
    sampled_plan = path_sampler.getSampledPlan(sampling_distance, *msg);
    
    //ROS_INFO("Received new path with %lu poses", sampled_plan.poses.size());
    sampled_plan.header.frame_id = msg->header.frame_id;
    plan_pub_.publish(sampled_plan);
    goal_pose = sampled_plan.poses.back(); // l'ultimo goal è l'ultimo della path

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

    for(int theta = -180; theta < 180; theta += delta){
        for(double r = 0.0; r < max_radius; r+=0.05){

            double x = r*cos(theta*M_PI/180.0 + yaw );
            double y = r*sin(theta*M_PI/180.0 + yaw );
            
            int x_cell = x/costmap_.resolution + costmap_.width/2;
            int y_cell = y/costmap_.resolution + costmap_.height/2;
           
            if(0 < x_cell && x_cell < costmap_.width && 0 < y_cell && y_cell < costmap_.height){
                int indx = y_cell * costmap_.width + x_cell;

                if(costmap_.occupancy_data[indx] > cost_obstacle){
                    polar_map[theta+180] = r; //rispetto footprint
                    break;
                }
            }
        }
    }

}


sensor_msgs::ImagePtr NavigationStack::eigenMatrixToImageMsg(const Eigen::MatrixXd& mat) {
    // Normalizza valori in [0,255]
    Eigen::MatrixXd norm = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
    double max = mat.cwiseAbs().maxCoeff();
    if (max > 1.0){
        norm = mat/max;
    }
    norm = (norm.array()-norm.minCoeff())*255;
    
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

void NavigationStack::util_sendField(Eigen::MatrixXd omegaAll){
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
}

void NavigationStack::run(){

    ros::Rate r(30);
    ros::Time last_goal = ros::Time::now();
    double ang_vel0 = 0.0;
<<<<<<< HEAD

=======
>>>>>>> d83c168ade52c077974e06b2dff40b77af4ce8ce
    while(ros::ok()){
        if (new_costmap) {

            buildPolarMap();
            util_sendLaserMsg();

            double lin_vel = 0.0;
            double ang_vel = 0.0;
<<<<<<< HEAD
            
            vs->reset();
=======
            
            vs->emptySet();
            vs->setRepellors(polar_map);
            //Eigen::MatrixXd omegaAll = vs->computeOmegaAll();
            
            /*std::ofstream file("matrice.csv");
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
            }*/
            //img_pub.publish(eigenMatrixToImageMsg(omegaAll));
            ang_vel += vs->getAngularVelocity(1.0);
            lin_vel = vs->getLinearVelocity();
>>>>>>> d83c168ade52c077974e06b2dff40b77af4ce8ce

            geometry_msgs::PoseStamped goal_in_fp;
            if(new_plan){
                ts = tf_.lookupTransform("base_footprint", goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
                tf2::doTransform(goal_pose, goal_in_fp, ts); // trasformo goal in odom frame
                
                vs->setAttractors(goal_in_fp);
                ang_vel += vs->getAngularVelocity(strength_attractors_angular_velocity);
            }
<<<<<<< HEAD
            ros::Time start = ros::Time::now();
            vs->setRepellors(polar_map);
            ROS_INFO("Time to set repellors: %f ms", (ros::Time::now() - start).toSec()*1000.0);
            
            ang_vel += vs->getAngularVelocity(1.0);
            lin_vel = vs->getLinearVelocity();

            geometry_msgs::Twist cmd_vel;
            if(ns_active){
=======
            geometry_msgs::Twist cmd_vel;
            if(ns_active){

>>>>>>> d83c168ade52c077974e06b2dff40b77af4ce8ce
                
                if(new_plan){
                    //ROS_INFO("got new plan");
                    cmd_vel.linear.x = lin_vel*linear_scaling;
                    //cmd_vel.linear.x = 0.0;
                }
                else{
                    cmd_vel.linear.x = 0.0;
                }
                cmd_vel.angular.z = (ang_vel0+ang_vel)*angular_scaling;//std::max(std::min(1.0, ang_vel*angular_scaling), -1.0);
                
                velocity_pub_.publish(cmd_vel);
            }
<<<<<<< HEAD
            
=======
            ROS_INFO("angular vel: %f", (ang_vel0+ang_vel)*angular_scaling);
>>>>>>> d83c168ade52c077974e06b2dff40b77af4ce8ce
            ang_vel0 = ang_vel;
            new_costmap = false;
            

        }

        ros::spinOnce();
        r.sleep();

    }

}

}


//Eigen::MatrixXd omegaAll = vs->computeOmegaAll();
            
            /*std::ofstream file("matrice.csv");
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
            }*/
            //img_pub.publish(eigenMatrixToImageMsg(omegaAll));