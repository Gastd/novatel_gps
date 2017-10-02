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
        void init();
        void init(std::string port);
        void close();
        void receiveDataFromGPS(sensor_msgs::NavSatFix*);
        ~GPS();

    private:
        int readDataFromReceiver();
        void configure();
        void command(const char* command);
        int getApproxTime();
        void decode(unsigned short msg_id);
        void gtime(unsigned char t_status, unsigned short t_week, unsigned long t_ms);
        void throwSerialComException(int);
        void waitReceiveInit();
        void waitFirstFix();
        void print_formatted();

        std::string serial_port_;
        // GPS data packet
        const int GPS_PACKET_SIZE;
        std::vector<unsigned char> gps_data_;
        // unsigned char gps_data_[GPS_PACKET_SIZE];

        double status_;
        double position_status_;    // TO DO: implement gps_state, gps_p_status, v_status
        double velocity_status_;
        unsigned short solution_status_;
        unsigned short position_type_;
        unsigned char number_sat_track_;
        unsigned char number_sat_sol_;
        double latitude_;
        double longitude_;
        double altitude_;
        // standard deviation provided gps receiver
        float  stdev_latitude_;
        double covar_latitude_;
        float  stdev_longitude_;
        double covar_longitude_;
        float  stdev_altitude_;
        double covar_altitude_;
        std::vector<double> velocity_;
        std::vector<double> sigma_position_;
        std::vector<double> sigma_velocity_;

        unsigned long number_satellites_;
        unsigned long gps_prn_;
        double x_;
        double y_;
        double z_;
        double clk_correction_;
        double ion_correction_;
        double trp_correction_;

        float cutoff_;
        long channels_;
        short prn_;
        unsigned long trk_stat_;
        double psr_;
        float doppler_;
        float CN0;

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
        int SATXYZ;
        int TRACKSTAT;

        enum STATES
        {
            // GPS States
            GPS_SYNC_ST,  //      0
            GPS_HEADER_ST,  //    1
            GPS_PAYLOAD_ST,  //   2
            GPS_CRC_ST,  //       3
        };

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

        /* BESTPOS */
        int BESTPOS_SOLSTAT;
        int BESTPOS_POSTYPE;
        int BESTPOS_LAT;
        int BESTPOS_LONG;
        int BESTPOS_HGT;
        int BESTPOS_UND;
        int BESTPOS_DATUMID;
        int BESTPOS_SLAT;
        int BESTPOS_SLON;
        int BESTPOS_SHGT;
        int BESTPOS_STNID;
        int BESTPOS_DIFFAGE;
        int BESTPOS_SOLAGE;
        int BESTPOS_SV;
        int BESTPOS_SOLNSV;
        int BESTPOS_GGL1;
        int BESTPOS_GGL1L2;
        int BESTPOS_EXTSOLSTAT;
        int BESTPOS_SIGMASK;

        /* BESTXYZ */
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

        /* SATXYZ */
        int SATXYZ_NSAT;
        int SATXYZ_PRN;
        int SATXYZ_X;
        int SATXYZ_Y;
        int SATXYZ_Z;
        int SATXYZ_CLKCORR;
        int SATXYZ_IONCORR;
        int SATXYZ_TRPCORR;

        /* TRACKSTAT */
        int TRACKSTAT_SOLSTAT;
        int TRACKSTAT_POSTYPE;
        int TRACKSTAT_CUTOFF;
        int TRACKSTAT_CHAN;
        int TRACKSTAT_PRN;
        int TRACKSTAT_TRK_STAT;
        int TRACKSTAT_PSR;
        int TRACKSTAT_DOPPLER;
        int TRACKSTAT_CN0;

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
