// Serial Port Headers (serialcom-termios)
#include <serialcom.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "novatel_gps.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "novatel_gps");
    ros::NodeHandle nh("gps");
    ros::Publisher gps_pub;

    GPS gps;

    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("data", 10);
    sensor_msgs::NavSatFix nav_msg;

    while(ros::ok())
    {
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.header.frame_id = "gps_frame";
        nav_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps.receiveDataFromGPS(nav_msg);

        gps_pub.publish(nav_msg);

        // gps_get_data(&gps_values);

        // nav_msg.header.stamp = ros::Time::now();
        // nav_msg.header.frame_id = "gps_frame";
        // nav_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        // nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // nav_msg.latitude = gps_values.p[0];
        // nav_msg.longitude = gps_values.p[1];
        // nav_msg.altitude = gps_values.p[2];

        // nav_msg.position_covariance[0] = gps_values.sigma_p[0];
        // nav_msg.position_covariance[4] = gps_values.sigma_p[1];
        // nav_msg.position_covariance[8] = gps_values.sigma_p[2];

        // nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        // gps_pub.publish(nav_msg);
    }

    return 0;
}
