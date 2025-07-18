#include <nav_master/set_end_point.h>


namespace set_end_point{


    setEndPoint::setEndPoint(ros::NodeHandle& nh) : tfListener(tfBuffer) {
        
        joy_sub_ = nh.subscribe("/joy", 1, &setEndPoint::JoyCallback, this);
        end_point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/end_point", 1);
        client_endPoint = nh.serviceClient<nav_master::endPoint>("/end_point_service");

        v = 0.0;
        s = 0.0;
        raggio = 2;
        end_point_current.pose.position.x = raggio;
        end_point_current.pose.position.y = 0.0;
    }

    setEndPoint::~setEndPoint(){}

    void setEndPoint::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg){
        double lr = msg->axes[0];
        v = lr*0.1;
        if (msg->buttons[5] == 1){
                v = 0.0;
            end_point_current.pose.position.y = 0.0;
        }    
    }


    void setEndPoint::update(){
        s = v*1.0; //è s = v*t con t= 1.0;
        double theta1 = s/raggio;
        double theta0 = std::atan2(end_point_current.pose.position.y, end_point_current.pose.position.x);
        double new_theta = std::max(-M_PI/2, std::min(M_PI/2, theta0 + theta1));
        double new_x =raggio*cos(new_theta);
        double new_y = raggio*sin(new_theta);
        
        geometry_msgs::PoseStamped new_end_point;
        new_end_point.header.frame_id = "base_footprint";
        new_end_point.header.stamp = ros::Time::now();
        new_end_point.pose.orientation.w = 1.0;
        new_end_point.pose.position.x = new_x;
        new_end_point.pose.position.y = new_y;
        end_point_current = new_end_point;

        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("odom", new_end_point.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(new_end_point, new_end_point_odom, transformStamped);
        end_point_pub_.publish(new_end_point_odom);
    }


    void setEndPoint::sendEndPointRequest(){

        srv.request.end_point.header = new_end_point_odom.header;
        srv.request.end_point.pose = new_end_point_odom.pose;

        if(client_endPoint.call(srv)){
            if (srv.response.success){
                ROS_INFO("End point sent!"); }
        }
        else{
            ROS_ERROR("Failed to send end point");
        }
    }

    void setEndPoint::run(){
        ros::Rate r(20);
        ros::Time last_goal = ros::Time::now();
        while(ros::ok()){
            update();
            if(ros::Time::now() - last_goal > ros::Duration(3.0)){
                sendEndPointRequest();   
                last_goal = ros::Time::now();        
            }
            ros::spinOnce();
            r.sleep();
        }
    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_end_point");
    ros::NodeHandle nh;
    set_end_point::setEndPoint ep(nh);
    ep.run();
    return 0;
}