#include <signal.h>

// Serial Port Headers (serialcom-termios)
#include <serialcom.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

// Project Headers
#include "main.h"
#include "gps.h"

int gps_ok = 0;

gps_t gps_values;

void setup_sig_handler();
void sig_handler(int sig);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "novatel_gps");
    ros::NodeHandle n("gps");
    ros::Publisher gps_pub;

    gps_pub = n.advertise<sensor_msgs::NavSatFix>("data", 10);

    int gps_new;

    sensor_msgs::NavSatFix nav_msg;

    // Signal catching
    setup_sig_handler();

    // Initialize GPS/gps
    if(!init_all())
        return 1;
    
    // Print out running time and update MAT file every second
    while(ros::ok())
    {   
        printf("gps\n");
        gps_new = gps_get_data(&gps_values);
        // gps_print_formatted(&gps_values);

        nav_msg.header.stamp = ros::Time::now();
        nav_msg.header.frame_id = "gps_frame";
        nav_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        nav_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        nav_msg.latitude = gps_values.p[0];
        nav_msg.longitude = gps_values.p[1];
        nav_msg.altitude = gps_values.p[2];

        nav_msg.position_covariance[0] = gps_values.sigma_p[0];
        nav_msg.position_covariance[4] = gps_values.sigma_p[1];
        nav_msg.position_covariance[8] = gps_values.sigma_p[2];

        nav_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gps_pub.publish(nav_msg);
    }
    
    // Clean up and close GPS/gps
    if(!close_all())
        return 1;
        
    return 0;
}

int init_all()
{
    // Initialize and open gps serial port
    if((gps_ok = gps_init((char*)"/dev/ttyUSB0")) != 1)
        ROS_WARN("could not initialize gps!");
    else
        ROS_INFO("gps initialized...");
    
    // If nothing is working, fails
    if(!gps_ok)
        return 0;
    
    return 1;
}

int close_all()
{
    // Clean up and close gps serial port
    if(!gps_close())
        ROS_WARN("Could not close gps!");
    else
        ROS_INFO("gps closed.");

    ROS_INFO("Goodbye!");
    
    return 1;
}

void sig_handler(int sig)
{
    ROS_INFO("Interrupted! Closing...");
    
    // Close everything cleanly before exiting
    if(!close_all())
    {
        ROS_FATAL("could not close cleanly!");
        exit(1);
    }
    
    exit(0);
}

void setup_sig_handler()
{
    signal(SIGINT, &sig_handler);
    signal(SIGTSTP, &sig_handler);
    signal(SIGABRT, &sig_handler);
    signal(SIGTERM, &sig_handler);
}
