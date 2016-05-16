#include "novatel_gps.h"

/*************************** CRC functions (Firmware Reference Manual, p.32 + APN-030 Rev 1 Application Note) ***************************/

inline unsigned long ByteSwap (unsigned long n)
{ 
   return ( ((n &0x000000FF)<<24) + ((n &0x0000FF00)<<8) + ((n &0x00FF0000)>>8) + (( n &0xFF000000)>>24) ); 
}

#define CRC32_POLYNOMIAL    0xEDB88320L

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
inline unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
inline unsigned long CalculateBlockCRC32
(
    unsigned long ulCount,  /* Number of bytes in the data block */
    unsigned char *ucBuffer /* Data block */
)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ((ulCRC) ^ (*ucBuffer)) ) & 0xff );
        ucBuffer++;
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    
    return( ulCRC );
}


// GPS Class methods

GPS::GPS()
{
    serial_port = "/dev/ttyUSB0";
    gps_week = 0;
    gps_secs = 0;

    D_SYNC0 = 0xAA;
    D_SYNC1 = 0x44;
    D_SYNC2 = 0x12;
    D_HDR_LEN = 28;

    BXYZ_PSTAT    = (D_HDR_LEN);
    BXYZ_PTYPE    = (D_HDR_LEN+4);
    BXYZ_PX       = (D_HDR_LEN+8);
    BXYZ_PY       = (D_HDR_LEN+16);
    BXYZ_PZ       = (D_HDR_LEN+24);
    BXYZ_sPX      = (D_HDR_LEN+32);
    BXYZ_sPY      = (D_HDR_LEN+36);
    BXYZ_sPZ      = (D_HDR_LEN+40);
    BXYZ_VSTAT    = (D_HDR_LEN+44);
    BXYZ_VTYPE    = (D_HDR_LEN+48);
    BXYZ_VX       = (D_HDR_LEN+52);
    BXYZ_VY       = (D_HDR_LEN+60);
    BXYZ_VZ       = (D_HDR_LEN+68);
    BXYZ_sVX      = (D_HDR_LEN+76);
    BXYZ_sVY      = (D_HDR_LEN+80);
    BXYZ_sVZ      = (D_HDR_LEN+84);
    BXYZ_STNID    = (D_HDR_LEN+88);
    BXYZ_VLATE    = (D_HDR_LEN+92);
    BXYZ_DIFFAGE  = (D_HDR_LEN+96);
    BXYZ_SOLAGE   = (D_HDR_LEN+92);
    BXYZ_SV       = (D_HDR_LEN+92);
    BXYZ_SOLSV    = (D_HDR_LEN+92);

    S_MSG_ID     = 2;
    S_MSG_LEN    = 2;
    S_SEQ_NUM    = 2;
    S_T_WEEK     = 2;
    S_T_MS       = 4;
    S_GPS_STATUS = 4;
    S_RESERVED   = 2;
    S_SW_VERS    = 2;
    S_CRC        = 4;

    TIMEOUT_US = 100000;
    OLD_BPS    = 9600;
    BPS        = 115200;
    MAX_BYTES  = 500;

    BESTPOS = 42;
    GPGGA   = 218;
    GPGSA   = 221;
    GPRMC   = 225;
    BESTXYZ = 241;

    init();
}

GPS::~GPS()
{
    close();
}

void GPS::init()
{
    int err;

    // Init serial port at 9600 bps
    if((err = serialcom_init(&gps_SerialPortConfig, 1, (char*)serial_port.c_str(), OLD_BPS)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
        throwSerialComException(err);
    }

    // Configure GPS, set baudrate to 115200 bps and reconnect
    configure();

    // Request GPS data
    command("LOG BESTXYZB ONTIME 0.05");
    ROS_INFO("gps initialized...");    
}

int GPS::readDataFromReceiver()
{
    int i;
    int err;
    int data_ready = 0;
    
    // State machine variables
    int b = 0, bb = 0, s = GPS_SYNC_ST;
    
    // Storage for data read from serial port
    unsigned char data_read;
    
    // Multi-byte data
    unsigned short msg_id, msg_len, t_week;
    unsigned long t_ms, crc_from_packet;

    //command("LOG BESTXYZB ONCE");

    // Try to sync with IMU and get latest data packet, up to MAX_BYTES read until failure
    for(i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
    {
        // Read data from serial port
        if((err = serialcom_receivebyte(&gps_SerialPortConfig, &data_read, TIMEOUT_US)) != SERIALCOM_SUCCESS)
        {
                ROS_ERROR_STREAM("serialcom_receivebyte failed " << err);
                continue;
        }
        
        // Parse GPS packet (Firmware Reference Manual, p.22)
        switch(s)
        {
            case GPS_SYNC_ST:
            {
                // State logic: Packet starts with 3 sync bytes with values 0xAA, 0x44, 0x12
                switch(b)
                {
                    case SYNC0:
                    {
                        if(data_read == D_SYNC0)
                        {
                            gps_data_[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }

                    case SYNC1:
                    {
                        if(data_read == D_SYNC1)
                        {
                            gps_data_[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }

                    case SYNC2:
                    {
                        if(data_read == D_SYNC2)
                        {
                            gps_data_[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }
                }
                // State transition: I have reached the HDR_LEN byte without resetting
                if(b == HDR_LEN)
                    s = GPS_HEADER_ST;
            }
            break;

            case GPS_HEADER_ST:
            {
                // State logic: HDR_LEN, MSG_ID, MSG_TYPE, PORT_ADDR, MSG_LEN, SEQ_NUM, IDLE_T, T_STATUS, T_WEEK, T_MS, GPS_STATUS, RESERVED, SW_VERS
                switch(b)
                {
                    case HDR_LEN:
                    {
                        if(data_read == D_HDR_LEN)
                        {
                            gps_data_[b] = data_read;
                            b++;
                        }
                        else
                        {
                            // Invalid HDR_LEN, reset
                            ROS_ERROR_STREAM("invalid HDR_LEN " << data_read);
                                b = 0;
                            s = GPS_SYNC_ST;
                        }
                        break;
                    }

                    case MSG_ID:
                    {                       
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_ID)
                        {
                            // Merge bytes and process
                            memcpy((void*)&msg_id, (void*)&gps_data_[MSG_ID], sizeof(unsigned short));                               
                            
                            // Update byte indices
                            bb = 0;
                            b += S_MSG_ID;
                        }
                        break;
                    }

                    case MSG_TYPE:
                    {
                        gps_data_[b] = data_read;
                        b++;
                        break;
                    }

                    case PORT_ADDR:
                    {
                        gps_data_[b] = data_read;
                        b++;
                        break;
                    }

                    case MSG_LEN:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_LEN)
                        {
                            // Merge bytes and process
                            memcpy((void*)&msg_len, (void*)&gps_data_[MSG_LEN], sizeof(unsigned short));

                            // I was having some problems with (msg_len == 0)...
                            if(msg_len != 0)
                            {                   
                                // Update byte indices
                                bb = 0;
                                b += S_MSG_LEN;
                            }
                            else
                            {                               
                                // Something wrong, reset
                                bb = 0;
                                b = 0;
                                s = GPS_SYNC_ST;
                            }
                        }
                        break;
                    }

                    case SEQ_NUM:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_SEQ_NUM)
                        {
                            // Merge bytes and process
                            //memcpy((void*)&seq_num, (void*)&gps_data_[SEQ_NUM], sizeof(unsigned short));   
                            
                            // Update byte indices
                            bb = 0;
                            b += S_SEQ_NUM;
                        }
                        break;
                    }

                    case IDLE_T:
                    {
                        gps_data_[b] = data_read;
                        b++;                    
                        break;
                    }

                    case T_STATUS:
                    {
                        gps_data_[b] = data_read;
                        b++;                    
                        break;
                    }

                    case T_WEEK:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_WEEK)
                        {
                            // Merge bytes and process
                            memcpy((void*)&t_week, (void*)&gps_data_[T_WEEK], sizeof(unsigned short));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_T_WEEK;
                        }
                        break;
                    }

                    case T_MS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_MS)
                        {
                            // Merge bytes and process
                            memcpy((void*)&t_ms, (void*)&gps_data_[T_MS], sizeof(unsigned long));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_T_MS;
                        }
                        break;
                    }

                    case GPS_STATUS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_GPS_STATUS)
                        {
                            // Merge bytes and process
                            memcpy((void*)&(status_), (void*)&gps_data_[GPS_STATUS], sizeof(unsigned long));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_GPS_STATUS;
                        }
                        break;
                    }

                    case RESERVED:
                    {   
                        // Index bb is for bytes in multi-byte variables
                        // Skip this section, no useful data
                        bb++;
                        
                        if(bb == S_RESERVED)
                        {
                            // Update byte indices
                            bb = 0;
                            b += S_RESERVED;
                        }
                        break;
                    }

                    case SW_VERS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data_[b+bb] = data_read;
                        bb++;

                        if(bb == S_SW_VERS)
                        {
                            // Merge bytes and process
                            //memcpy((void*)&sw_vers, (void*)&gps_data_[SW_VERS], sizeof(unsigned short));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_SW_VERS;
                        }
                        break;
                    }
                }

                // State transition: I have reached the DATA bytes without resetting
                if(b == DATA)
                    s = GPS_PAYLOAD_ST;  
            }
            break;

            case GPS_PAYLOAD_ST:
            {
                // State logic: Grab data until you reach the CRC bytes
                gps_data_[b+bb] = data_read;
                bb++;
                
                // State transition: I have reached the CRC bytes
                if(bb == msg_len)
                {
                    // Bytes are decoded after CRC check
                                        
                    // Update byte indices
                    bb = 0;
                    b += msg_len;
                    s = GPS_CRC_ST;
                }
            }
            break;

            case GPS_CRC_ST:
            {
                // Index bb is for bytes in multi-byte variables
                gps_data_[b+bb] = data_read;
                bb++;
                if(bb == S_CRC)
                {       
                    unsigned long crc_calculated;
                    // Grab CRC from packet
                    crc_from_packet = ((unsigned long)((gps_data_[b+3] << 24) | (gps_data_[b+2] << 16) | (gps_data_[b+1] << 8) | gps_data_[b]));
                    
                    // Calculate CRC from packet (b = packet size)
                    crc_calculated = CalculateBlockCRC32(b, gps_data_);
                    
                    // Compare them to see if valid packet 
                    if(0/*crc != ByteSwap(CRC)*/)
                    // if(crc_from_packet != ByteSwap(crc_calculated))
                    {
                        ROS_ERROR("CRC does not match (%0lx != %0lx)", crc_from_packet, crc_calculated);
                    }
                    else
                    {
                        decode(msg_id);
                        data_ready = 1;
                    }                   
                        
                    // State transition: Unconditional reset
                    bb = 0;
                    b = 0;
                    s = GPS_SYNC_ST;
                }
            }
            break;
        }
    }

    // Flush port and request more data
    // tcflush(gps_SerialPortConfig.fd, TCIOFLUSH);
    // command("LOG BESTXYZB ONCE");
    return data_ready;
}

void GPS::decode(unsigned short msg_id)
{   
    if(sizeof(double) != 8)
        ROS_ERROR("sizeof(double) != 8, check decode");

    if(msg_id == BESTXYZ)
    {
        memcpy((void*)&p_status_, (void*)&gps_data_[BXYZ_PSTAT], sizeof(long));
        memcpy((void*)&v_status_, (void*)&gps_data_[BXYZ_VSTAT], sizeof(long));

        if((p_status_ == 0)&&(v_status_ == 0))
            ROS_INFO("GPS: Solution computed");
        else
            ROS_WARN("GPS: Solution invalid (WRONG VALUES: %ld %ld)", (long)p_status_, (long)v_status_);

        memcpy((void*)&p_[0], (void*)&gps_data_[BXYZ_PX], sizeof(double));
        memcpy((void*)&p_[1], (void*)&gps_data_[BXYZ_PY], sizeof(double));
        memcpy((void*)&p_[2], (void*)&gps_data_[BXYZ_PZ], sizeof(double));

        memcpy((void*)&v_[0], (void*)&gps_data_[BXYZ_VX], sizeof(double));
        memcpy((void*)&v_[1], (void*)&gps_data_[BXYZ_VY], sizeof(double));
        memcpy((void*)&v_[2], (void*)&gps_data_[BXYZ_VZ], sizeof(double));

        memcpy((void*)&sigma_p_[0], (void*)&gps_data_[BXYZ_sPX], sizeof(double));
        memcpy((void*)&sigma_p_[1], (void*)&gps_data_[BXYZ_sPY], sizeof(double));
        memcpy((void*)&sigma_p_[2], (void*)&gps_data_[BXYZ_sPZ], sizeof(double));

        memcpy((void*)&sigma_v_[0], (void*)&gps_data_[BXYZ_sVX], sizeof(double));
        memcpy((void*)&sigma_v_[1], (void*)&gps_data_[BXYZ_sVY], sizeof(double));
        memcpy((void*)&sigma_v_[2], (void*)&gps_data_[BXYZ_sVZ], sizeof(double));
    }
}

void GPS::print_formatted()
{
    ROS_INFO("GPS: p(%.3lf %.3lf %.3lf) sp(%.3lf %.3lf %.3lf) v(%.3lf %.3lf %.3lf) sv(%.3lf %.3lf %.3lf) status(%lx %lx)\n",
            p_[0], p_[1], p_[2], sigma_p_[0], sigma_p_[1], sigma_p_[2], v_[0], v_[1], v_[2], sigma_v_[0], sigma_v_[1], sigma_v_[2], (long)p_status_, (long)v_status_);
}


void GPS::close()
{
    int err;

    if((err = serialcom_close(&gps_SerialPortConfig)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_close failed " << err);
        ROS_WARN("Could not close gps!");
        throwSerialComException(err);
    }
    ROS_INFO("gps closed.");
    ROS_INFO("Goodbye!");
}

void GPS::configure()
{
    int err;

    // GPS should be configured to 9600 and change to 115200 during execution
    command("COM COM1,115200,N,8,1,N,OFF,ON");
    // command("COM COM2,115200,N,8,1,N,OFF,ON");
    
    if((err = serialcom_close(&gps_SerialPortConfig)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_close failed " << err);
        throwSerialComException(err);
    }

    // Reconnecting at 115200 bps
    if((err = serialcom_init(&gps_SerialPortConfig, 1, (char*)serial_port.c_str(), BPS)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
        throwSerialComException(err);
    }

    // GPS time should be set approximately
    if(!getApproxTime())
    {
        ROS_ERROR("could not set approximate time");
    }
    else
    {
        char buf[100];
        sprintf(buf, "SETAPPROXTIME %lu %lu", gps_week, gps_secs);
        command(buf);
    }

    // GPS position should be set approximately (hard coded to LARA/UnB coordinates)
    command("SETAPPROXPOS -15.765824 -47.872109 1024");
}

void GPS::receiveDataFromGPS(sensor_msgs::NavSatFix& output)
{
    readDataFromReceiver();
    output.latitude = p_[0];
    output.longitude = p_[1];
    output.altitude = p_[2];

    output.position_covariance[0] = sigma_p_[0];
    output.position_covariance[4] = sigma_p_[1];
    output.position_covariance[8] = sigma_p_[2];

    output.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}


void GPS::throwSerialComException(int err)
{
    switch(err)
    {
        case SERIALCOM_ERROR_IOPL:
        {
            throw std::runtime_error("serialcom error iopl");
            break;
        }
        case SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION:
        {
            throw std::runtime_error("serialcom error max wait end of transmission");
            break;
        }
        case SERIALCOM_ERROR_MAXWAITFORRECEPTION:
        {
            throw std::runtime_error("serialcom error max wait for reception");
            break;
        }
        case SERIALCOM_ERROR_MAXBPSPRECISION:
        {
            throw std::runtime_error("serialcom error max bps precision");
            break;
        }
        case SERIALCOM_ERROR_INCORRECTPORTNUMBER:
        {
            throw std::runtime_error("serialcom error incorrect port number");
            break;
        }
        case SERIALCOM_ERROR_INVALIDBAUDRATE:
        {
            throw std::runtime_error("serialcom error invalid baudrate");
            break;
        }
        case SERIALCOM_ERROR_INVALIDDEVICE:
        {
            throw std::runtime_error("serialcom error invalid device");
            break;
        }
    }
}

void GPS::command(const char* command)
{
    int i;
    int len = strlen(command);

    // Echo
    ROS_INFO("Sending command: %s\n", command);
    for(i = 0; i < len; i++)
    {
        serialcom_sendbyte(&gps_SerialPortConfig, (unsigned char*) &command[i]);
        usleep(5000);
    }

    serialcom_sendbyte(&gps_SerialPortConfig, (unsigned char*) "\r");
    usleep(5000);
    serialcom_sendbyte(&gps_SerialPortConfig, (unsigned char*) "\n");
    usleep(5000);
}

// Calculate GPS week number and seconds, within 10 minutes of actual time, for initialization
int GPS::getApproxTime()
{
    // Time difference between January 1, 1970 and January 6, 1980
    // Source: http://www.timeanddate.com/date/durationresult.html?d1=1&m1=jan&y1=1970&h1=0&i1=0&s1=0&d2=6&m2=jan&y2=1980&h2=0&i2=0&s2=0
    const unsigned long int time_diff = 315964800;

    // # of seconds in a week
    const unsigned long int secs_in_week = 604800;

    // Unix time
    time_t cpu_secs;

    // Unprocessed GPS time
    unsigned long int gps_time;

    // Get time
    cpu_secs = time(NULL);

    // Offset to GPS time and calculate weeks and seconds
    gps_time = cpu_secs - time_diff;
    gps_week = gps_time / secs_in_week;
    gps_secs = gps_time % secs_in_week;

    if((gps_week != 0) && (gps_secs != 0))
        return 1;
    else
        return 0;
}

void GPS::gtime(unsigned char t_status, unsigned short t_week, unsigned long t_ms)
{
    // GPS week number = full week number starting from midnight of the night from January 5 to January 6, 1980
    // TO DO: figure out what this was for. I think it was supposed to decode time information and know if GPS has valid time
}

