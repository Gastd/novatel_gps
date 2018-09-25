#ifndef NOVATEL_GPS_H
#define NOVATEL_GPS_H

#include <vector>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "novatel_gps/GpsXYZ.h"
#include "novatel_gps/MsgHeader.h"
#include "novatel_gps/SatXYZ.h"
#include "novatel_gps/TrackStat.h"
#include "novatel_gps/Range.h"
#include "novatel_gps/LogAll.h"

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

class GPS
{
public:
    GPS();
    void init(int log_id);
    void init(int log_id, std::string port, double rate);
    void close();
    void receiveDataFromGPS(sensor_msgs::NavSatFix*);
    void receiveDataFromGPS(novatel_gps::GpsXYZ*);
    void receiveDataFromGPS(novatel_gps::LogAll*, novatel_gps::GpsXYZ*);
    ~GPS();

    /* Log Message IDs */
    const int BESTPOS   = 42;
    const int BESTXYZ   = 241;
    const int RANGE     = 43;
    const int SATXYZ    = 270;
    const int TRACKSTAT = 83;

private:
    int readDataFromReceiver();
    void configure();
    void command(const char* command);
    int getApproxTime();
    void decode(uint16_t msg_id);
    void throwSerialComException(int);
    void waitReceiveInit();

    novatel_gps::MsgHeader msg_header_;
    novatel_gps::SatXYZ satellites_;
    novatel_gps::TrackStat tracking_;
    novatel_gps::Range pseudorange_;

    std::string serial_port_;
    // GPS data packet
    const int GPS_PACKET_SIZE;
    std::vector<uint8_t> gps_data_;

    uint8_t time_stat_;
    double status_;
    uint16_t position_status_;    // TO DO: implement gps_state, gps_p_status, v_status
    uint16_t velocity_status_;
    uint16_t velocity_type_;
    uint16_t solution_status_;
    uint16_t position_type_;
    uint8_t number_sat_track_;
    uint8_t number_sat_sol_;
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

    uint32_t number_satellites_;
    double x_;
    double y_;
    double z_;

    enum HEADER_ORDER
    {
        // Header byte order/format
        SYNC0,          //     0   int8_t
        SYNC1,          //     1   int8_t
        SYNC2,          //     2   int8_t
        HDR_LEN,        //     3   uint8_t
        MSG_ID,         //     4   uint16_t
        MSG_TYPE = 6,   //     6   int8_t
        PORT_ADDR,      //     7   uint8_t
        MSG_LEN,        //     8   uint16_t
        SEQ_NUM = 10,   //     10  uint16_t
        IDLE_T = 12,    //     12  uint8_t
        T_STATUS,       //     13  enum
        T_WEEK,         //     14  uint16_t
        T_MS = 16,      //     16  GPSec (uint32_t)
        GPS_STATUS = 20,//     20  uint32_t
        RESERVED = 24,  //     24  uint16_t
        SW_VERS = 26,   //     26  uint16_t
        DATA = 28,      //     28  variable
    };


    enum STATES
    {
        // GPS States
        GPS_SYNC_ST,    //  0
        GPS_HEADER_ST,  //  1
        GPS_PAYLOAD_ST, //  2
        GPS_CRC_ST,     //  3
    };

    // Serial port
    int TIMEOUT_US;
    int OLD_BPS;
    int BPS;
    int MAX_BYTES;
    int rate_;
    SERIALPORTCONFIG gps_SerialPortConfig_;

    // GPS week
    unsigned long gps_week_, gps_week_1024_;
    double gps_secs_;

    // Default values
    int D_SYNC0;
    int D_SYNC1;
    int D_SYNC2;
    int D_HDR_LEN;

    /* Message Header */
    uint16_t D_HDR;
    uint16_t D_MSG_ID;
    uint16_t D_MSG_TP;
    uint16_t D_PORT_ADD;
    uint16_t D_MSG_LEN;
    uint16_t D_SEQ;
    uint16_t D_IDLE_T;
    uint16_t D_TIME_ST;
    uint16_t D_G_WEEK;
    uint16_t D_G_MS;
    uint16_t D_RCV_ST;
    uint16_t D_RCV_SW_V;

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
    int SATXYZ_OFFSET;

    /* TRACKSTAT */
    int TRACKSTAT_SOLSTAT;
    int TRACKSTAT_POSTYPE;
    int TRACKSTAT_CUTOFF;
    int TRACKSTAT_CHAN;
    int TRACKSTAT_PRN;
    int TRACKSTAT_TRKSTAT;
    int TRACKSTAT_PSR;
    int TRACKSTAT_DOPPLER;
    int TRACKSTAT_CNo;
    int TRACKSTAT_LOCKTIME;
    int TRACKSTAT_PSRRES;
    int TRACKSTAT_REJECT;
    int TRACKSTAT_PSRW;
    int TRACKSTAT_OFFSET;

    /* RANGE */
    int RANGE_OBS;
    int RANGE_PRN;
    int RANGE_PSR;
    int RANGE_PSR_STD;
    int RANGE_ADR;
    int RANGE_ADR_STD;
    int RANGE_DOPPLER;
    int RANGE_CNo;
    int RANGE_LOCKTIME;
    int RANGE_TRKSTART;
    int RANGE_OFFSET;

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

#endif // NOVATEL_GPS_H
