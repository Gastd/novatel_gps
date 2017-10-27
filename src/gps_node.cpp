// Serial Port Headers (serialcom-termios)
#include <serialcom.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps/GpsXYZ.h>

#include "novatel_gps.h"

class GpsNode
{
private:
    GPS gps;
    sensor_msgs::NavSatFix gps_reading_;
    novatel_gps::GpsXYZ gps_xyz_reading_;

    std::string port;

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    ros::Publisher gps_data_pub_;
    // ros::ServiceServer calibrate_serv_;

    bool running;

    bool autocalibrate_;
    bool calibrate_requested_;
    bool calibrated_;

    int error_count_;
    int slow_count_;
    std::string was_slow_;
    std::string error_status_;

    int log_id_; 

    std::string frameid_;

    double desired_freq_;

public:
    GpsNode(ros::NodeHandle n) : node_handle_(n), private_node_handle_("~"), calibrate_requested_(false),
    error_count_(0), slow_count_(0), desired_freq_(50)
    {
        ros::NodeHandle gps_node_handle(node_handle_, "gps");
        private_node_handle_.param("port", port, std::string("/dev/ttyUSB0"));
        private_node_handle_.param("frame_id", frameid_, std::string("gps_frame"));
        // TODO: Remove magical number.
        private_node_handle_.param("log", log_id_, gps.BESTXYZ);

        if(log_id_ == gps.BESTPOS)
        {
            // init the publisher
            gps_data_pub_ = gps_node_handle.advertise<sensor_msgs::NavSatFix>("fix", 10);
        
            // init the message
            gps_reading_.header.frame_id = frameid_;
            gps_reading_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        }
        if(log_id_ == gps.BESTXYZ)
        {
            // init the publisher
            gps_data_pub_ = gps_node_handle.advertise<novatel_gps::GpsXYZ>("gps_cart", 10);
        }

        // calibrate_serv_ = gps_node_handle.advertiseService("calibrate", &GpsNode::calibrate, this);
        running = false;
    }

    void start()
    {
        try
        {
            gps.init(log_id_, port);
            ROS_INFO("GPS initialized...");
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Exception thrown while starting GPS. This sometimes happens if you are not connected " <<
                             "to an GPS or if another process is trying to access the GPS port. You may try 'lsof|grep "
                             << port.c_str() <<
                             "' to see if other processes have the port open."<< std::endl << e.what());
            ROS_WARN("Could not start GPS!");
            ROS_ERROR_STREAM(e.what());
            ROS_BREAK();
        }
    }

    bool spin()
    {
        start();
        while(ros::ok())
        {
            publishData();
            ros::spinOnce();
        }
        stop();
    }

    void publishData()
    {
        getData();
        gps_data_pub_.publish(gps_reading_);
    }

    void getData()
    {
        if(log_id_ == gps.BESTPOS)
        {
            gps.receiveDataFromGPS(&gps_reading_);
            gps_reading_.header.stamp = ros::Time::now();
        }
        if(log_id_ == gps.BESTXYZ)
        {
            gps.receiveDataFromGPS(&gps_xyz_reading_);
        }
    }

    void stop()
    {
        try
        {
            gps.close();
            ROS_INFO("GPS closed.");
        }
        catch(const std::exception& e)
        {
            ROS_WARN("Could not close GPS!");
            ROS_ERROR_STREAM(e.what());
        }
        ROS_INFO("Goodbye!");
    }

    ~GpsNode()
    {
        stop();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "novatel_gps");
    ros::NodeHandle n;

    GpsNode gpsn(n);
    gpsn.spin();

    return 0;
}
