#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>  // Untuk posisi GPS

// Callback untuk status drone
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    ROS_INFO("Drone Mode: %s | Armed: %d | Connected: %d", 
             msg->mode.c_str(), msg->armed, msg->connected);
}

// Callback untuk membaca posisi drone
void positionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    ROS_INFO("Drone Position: Lat: %f, Lon: %f, Alt: %f", 
             msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gcs_node");
    ros::NodeHandle nh;

    // Subscribe ke MAVROS untuk mendapatkan status dan posisi drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);
    ros::Subscriber pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, positionCallback);

    ros::spin();
    return 0;
}

