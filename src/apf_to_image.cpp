#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

Eigen::MatrixXd costmap;

void onReceiveMatrix(const Eigen::MatrixXd& mat) {
    costmap = mat;
}

sensor_msgs::ImagePtr eigenMatrixToImageMsg(const Eigen::MatrixXd& mat, const std::string& encoding = "mono8") {
    // Normalizza valori in [0,255]
    double min = mat.minCoeff();
    double max = mat.maxCoeff();
    Eigen::MatrixXd norm =  (mat.array() - min) / (max - min + 1e-8);  // evita divisione per zero
    norm = norm * 255.0;

    // Converti in cv::Mat (uint8_t)
    cv::Mat img(mat.rows(), mat.cols(), CV_8UC1); // mono8 = 8 bit, 1 canale

    for(int r = 0; r < mat.rows(); ++r) {
        for(int c = 0; c < mat.cols(); ++c) {
            img.at<uint8_t>(r, c) = static_cast<uint8_t>(norm(r, c));
        }
    }

    // Crea sensor_msgs::Image usando cv_bridge
    return cv_bridge::CvImage(std_msgs::Header(), encoding, img).toImageMsg();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_image_publisher");
    ros::NodeHandle nh;

    ros::Subscriber matrix_sub = nh.subscribe("/matrix", 1, &onReceiveMatrix);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("my_costmap_image", 1);

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        // Esempio: matrice 100x100 con valori casuali

        sensor_msgs::ImagePtr msg = eigenMatrixToImageMsg(costmap);

        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "map";  // se vuoi, anche "camera" o altro

        img_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
