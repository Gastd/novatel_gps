#ifndef NOVATEL_GPS_H
#define NOVATEL_GPS_H

#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

// #define GPS_PACKET_SIZE 200

namespace GPS_DEFINES
{
} // GPS_DEFINES

// namespace novatel_gps {
    class GPS
    {
    public:
        GPS();
        void receiveDataFromGPS(sensor_msgs::NavSatFix&);
        ~GPS();

    private:
        void init();
        int readDataFromReceiver();
        void configure();
        void command(const char* command);
        int getApproxTime();
        void decode(unsigned short msg_id);
        void gtime(unsigned char t_status, unsigned short t_week, unsigned long t_ms);
        void throwSerialComException(int);
        void print_formatted();
        void close();

        std::string serial_port;
        // GPS data packet
        const int GPS_PACKET_SIZE;
        std::vector<unsigned char> gps_data_;
        // unsigned char gps_data_[GPS_PACKET_SIZE];

        double status_;
        double position_status_;    // TO DO: implement gps_state, gps_p_status, v_status
        double velocity_status_;
        double latitude_;
        double longitude_;
        double altitude_;
        std::vector<double> velocity_;
        // standard deviation provided gps receiver
        std::vector<double> sigma_position_;
        std::vector<double> sigma_velocity_;

        enum HEADER_ORDER
        {
            // Header byte order/format
            SYNC0,         //     0   char
            SYNC1,         //     1   char
            SYNC2,         //     2   char
            HDR_LEN,       //     3   uchar
            MSG_ID,        //     4   ushort
            MSG_TYPE=6,    //     6   char
            PORT_ADDR,     //     7   uchar
            MSG_LEN,       //     8   ushort
            SEQ_NUM=10,    //     10  ushort
            IDLE_T=12,     //     12  uchar
            T_STATUS,      //     13  enum
            T_WEEK,        //     14  ushort
            T_MS=16,       //     16  GPSec (ulong)
            GPS_STATUS=20, //     20  ulong
            RESERVED=24,   //     24  ushort
            SW_VERS=26,    //     26  ushort
            DATA=28,       //     28  variable
        };

        /* Log Message IDs */
        int BESTPOS;
        int GPGGA;
        int GPGSA;
        int GPRMC;
        int BESTXYZ;

        enum STATES
        {
            // GPS States
            GPS_SYNC_ST,  //      0
            GPS_HEADER_ST,  //    1
            GPS_PAYLOAD_ST,  //   2
            GPS_CRC_ST,  //       3
        };

        // double p_[3];
        // double sigma_p_[3];
        // double v_[3];
        // double sigma_v_[3];

        // Serial port
        int TIMEOUT_US;
        int OLD_BPS;
        int BPS;
        int MAX_BYTES;
        SERIALPORTCONFIG gps_SerialPortConfig;

        // GPS week
        unsigned long gps_week;
        unsigned long gps_secs;

        // Default values
        int D_SYNC0;
        int D_SYNC1;
        int D_SYNC2;
        int D_HDR_LEN;

        int BXYZ_PSTAT;
        int BXYZ_PTYPE;
        int BXYZ_PX;
        int BXYZ_PY;
        int BXYZ_PZ;
        int BXYZ_sPX;
        int BXYZ_sPY;
        int BXYZ_sPZ;
        int BXYZ_VSTAT;
        int BXYZ_VTYPE;
        int BXYZ_VX;
        int BXYZ_VY;
        int BXYZ_VZ;
        int BXYZ_sVX;
        int BXYZ_sVY;
        int BXYZ_sVZ;
        int BXYZ_STNID;
        int BXYZ_VLATE;
        int BXYZ_DIFFAGE;
        int BXYZ_SOLAGE;
        int BXYZ_SV;
        int BXYZ_SOLSV;

        // Multi-byte sizes
        int S_MSG_ID;
        int S_MSG_LEN;
        int S_SEQ_NUM;
        int S_T_WEEK;
        int S_T_MS;
        int S_GPS_STATUS;
        int S_RESERVED;
        int S_SW_VERS;
        int S_CRC;
    };
// } // novatel_gps

// typedef ulong ulong;
// typedef uchar uchar;

#endif // NOVATEL_GPS_H
