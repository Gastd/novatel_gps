/**
 * @file      gps.c
 * @author    George Andrew Brindeiro
 * @date      03/12/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

/* Includes */

// General
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

// Project Headers
#include "main.h"
#include "gps.h"

/* Definitions */

// GPS States
#define GPS_SYNC_ST         0
#define GPS_HEADER_ST       1
#define GPS_PAYLOAD_ST      2
#define GPS_CRC_ST          3

// Serial port
#define GPS_SERIAL_PORT "/dev/ttyUSB0"
#define TIMEOUT_US      100000
#define BPS             115200
#define MAX_BYTES       500

SERIALPORTCONFIG gps_SerialPortConfig;

/* Global variables */

// GPS week
unsigned long gps_week = 0;
unsigned long gps_secs = 0;

int gps_init(char* serial_port)
{
    int err;

    // Check whether a port was defined, and enforce default value GPS_SERIAL_PORT
    if(serial_port == NULL)
        serial_port = (char*)GPS_SERIAL_PORT;

    // Init serial port at 9600 bps
    if((err = serialcom_init(&gps_SerialPortConfig, 1, serial_port, 9600)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
        return 0;
    }

    // Configure GPS, set baudrate to 115200 bps and reconnect
    gps_configure(serial_port);

    // Request GPS data
    gps_command("LOG BESTXYZB ONTIME 0.05");
    
    return 1;
}

int gps_get_data(gps_t* data)
{
    int i;
    int err;
    int data_ready = 0;
    
    // State machine variables
    int b = 0, bb = 0, s = GPS_SYNC_ST;
    
    // Storage for data read from serial port
    unsigned char data_read;
    
    // GPS data packet
    unsigned char gps_data[GPS_PACKET_SIZE];
    
    // Multi-byte data
    unsigned short msg_id, msg_len, /*seq_num,*/ t_week/*, sw_vers*/;
    unsigned long t_ms, crc_from_packet;

    //gps_command("LOG BESTXYZB ONCE");

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
                            gps_data[b] = data_read;
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
                            gps_data[b] = data_read;
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
                            gps_data[b] = data_read;
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
                            gps_data[b] = data_read;
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
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_ID)
                        {
                            // Merge bytes and process
                            memcpy((void*)&msg_id, (void*)&gps_data[MSG_ID], sizeof(unsigned short));                               
                            
                            // Update byte indices
                            bb = 0;
                            b += S_MSG_ID;
                        }
                        
                        break;
                    }
                    
                    case MSG_TYPE:
                    {
                        gps_data[b] = data_read;
                        b++;                    
                        
                        break;
                    }
                    
                    case PORT_ADDR:
                    {
                        gps_data[b] = data_read;
                        b++;                    
                        
                        break;
                    }
                    
                    case MSG_LEN:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_LEN)
                        {
                            // Merge bytes and process
                            memcpy((void*)&msg_len, (void*)&gps_data[MSG_LEN], sizeof(unsigned short));
                                                        
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
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_SEQ_NUM)
                        {
                            // Merge bytes and process
                            //memcpy((void*)&seq_num, (void*)&gps_data[SEQ_NUM], sizeof(unsigned short));   
                            
                            // Update byte indices
                            bb = 0;
                            b += S_SEQ_NUM;
                        }
                        
                        break;
                    }
                    
                    case IDLE_T:
                    {
                        gps_data[b] = data_read;
                        b++;                    
                        
                        break;
                    }
                    
                    case T_STATUS:
                    {
                        gps_data[b] = data_read;
                        b++;                    
                        
                        break;
                    }
                    
                    case T_WEEK:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_WEEK)
                        {
                            // Merge bytes and process
                            memcpy((void*)&t_week, (void*)&gps_data[T_WEEK], sizeof(unsigned short));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_T_WEEK;
                        }
                        
                        break;
                    }
                    
                    case T_MS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_MS)
                        {
                            // Merge bytes and process
                            memcpy((void*)&t_ms, (void*)&gps_data[T_MS], sizeof(unsigned long));
                            
                            // Update byte indices
                            bb = 0;
                            b += S_T_MS;
                        }
                        
                        break;
                    }
                    
                    case GPS_STATUS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_GPS_STATUS)
                        {
                            // Merge bytes and process
                            memcpy((void*)&(data->status), (void*)&gps_data[GPS_STATUS], sizeof(unsigned long));
                            
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
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_SW_VERS)
                        {
                            // Merge bytes and process
                            //memcpy((void*)&sw_vers, (void*)&gps_data[SW_VERS], sizeof(unsigned short));
                            
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
                gps_data[b+bb] = data_read;
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
                gps_data[b+bb] = data_read;
                bb++;

                if(bb == S_CRC)
                {       
                    unsigned long crc_calculated;
                    
                    // Grab CRC from packet
                    crc_from_packet = ((unsigned long)((gps_data[b+3] << 24) | (gps_data[b+2] << 16) | (gps_data[b+1] << 8) | gps_data[b]));
                    
                    // Calculate CRC from packet (b = packet size)
                    crc_calculated = CalculateBlockCRC32(b, gps_data);
                    
                    // Compare them to see if valid packet 
                    if(0/*crc != ByteSwap(CRC)*/)
                    {
                        ROS_ERROR("CRC does not match (%0lx != %0lx)\n", crc_from_packet, crc_calculated);
                        }
                    else
                    {
                        gps_decode(data, gps_data, msg_id);
                        
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
    // gps_command("LOG BESTXYZB ONCE");

    return data_ready;
}

void gps_decode(gps_t* data, unsigned char gps_data[], unsigned short msg_id)
{   
    if(sizeof(double) != 8)
        ROS_ERROR("sizeof(double) != 8, check gps_decode");
    
    if(msg_id == BESTXYZ)
    {           
        memcpy((void*)&data->p_status, (void*)&gps_data[BXYZ_PSTAT], sizeof(long));
        memcpy((void*)&data->v_status, (void*)&gps_data[BXYZ_VSTAT], sizeof(long));
        

          if((data->p_status == 0)&&(data->v_status == 0))
                ROS_INFO("GPS: Solution computed\n");
            else
                ROS_WARN("GPS: Solution invalid (WRONG VALUES: %ld %ld)\n", (long)data->p_status, (long)data->v_status);
 
        
        memcpy((void*)&data->p[0], (void*)&gps_data[BXYZ_PX], sizeof(double));
        memcpy((void*)&data->p[1], (void*)&gps_data[BXYZ_PY], sizeof(double));
        memcpy((void*)&data->p[2], (void*)&gps_data[BXYZ_PZ], sizeof(double));
            
        memcpy((void*)&data->v[0], (void*)&gps_data[BXYZ_VX], sizeof(double));
        memcpy((void*)&data->v[1], (void*)&gps_data[BXYZ_VY], sizeof(double));
        memcpy((void*)&data->v[2], (void*)&gps_data[BXYZ_VZ], sizeof(double));
        
        memcpy((void*)&data->sigma_p[0], (void*)&gps_data[BXYZ_sPX], sizeof(double));
        memcpy((void*)&data->sigma_p[1], (void*)&gps_data[BXYZ_sPY], sizeof(double));
        memcpy((void*)&data->sigma_p[2], (void*)&gps_data[BXYZ_sPZ], sizeof(double));
        
        memcpy((void*)&data->sigma_v[0], (void*)&gps_data[BXYZ_sVX], sizeof(double));
        memcpy((void*)&data->sigma_v[1], (void*)&gps_data[BXYZ_sVY], sizeof(double));
        memcpy((void*)&data->sigma_v[2], (void*)&gps_data[BXYZ_sVZ], sizeof(double));       
    }
}

int gps_close()
{
    int err;
    
    if((err = serialcom_close(&gps_SerialPortConfig)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_close failed " << err);
        return 0;
    }
    
    return 1;
}

void gps_configure(char* serial_port)
{
    int err;
    
    // Check whether a port was defined, and enforce default value GPS_SERIAL_PORT
    if(serial_port == NULL)
        serial_port = (char*)GPS_SERIAL_PORT;
    
    
    // GPS should be configured to 9600 and change to 115200 during execution
    gps_command("COM COM1,115200,N,8,1,N,OFF,ON");
    gps_command("COM COM2,115200,N,8,1,N,OFF,ON");
    
    if((err = serialcom_close(&gps_SerialPortConfig)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_close failed " << err);
    }
    
    // Reconnecting at 115200 bps
    if((err = serialcom_init(&gps_SerialPortConfig, 1, (char*)GPS_SERIAL_PORT, BPS)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
    }
    
    // GPS time should be set approximately
    if(!gps_get_approx_time())
    {
        ROS_ERROR("could not set approximate time");
    }
    else
    {
        char buf[100];
        
        sprintf(buf, "SETAPPROXTIME %lu %lu", gps_week, gps_secs);
        gps_command(buf);
    }
    
    // GPS position should be set approximately (hard coded to LARA/UnB coordinates)
    gps_command("SETAPPROXPOS -15.765824 -47.872109 1024");
}

void gps_command(const char* command)
{
    int i;
    int len = strlen(command);
    
    // Echo
    //printf("\tgps_command: %s\n", command);
        
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
int gps_get_approx_time()
{
    // Time difference between January 1, 1970 and January 6, 1980
    // Source: http://www.timeanddate.com/date/durationresult.html?d1=1&m1=jan&y1=1970&h1=0&i1=0&s1=0&d2=6&m2=jan&y2=1980&h2=0&i2=0&s2=0
    const unsigned long int time_diff = 315964800;
    
    // # of seconds in a week
    const unsigned long int secs_in_week = 604800;
    
    // Unix time
    time_t cpu_secs;
    
    // Unprocessed GPS time
    unsigned long int g_time;

    // Get time
    cpu_secs = time(NULL);
  
    // Offset to GPS time and calculate weeks and seconds
    g_time = cpu_secs - time_diff;
    gps_week = g_time / secs_in_week;
    gps_secs = g_time % secs_in_week;
    
    if((gps_week != 0) && (gps_secs != 0))
        return 1;
    else
        return 0;
}

void gps_time(unsigned char t_status, unsigned short t_week, unsigned long t_ms)
{
    // GPS week number = full week number starting from midnight of the night from January 5 to January 6, 1980
    // TO DO: figure out what this was for. I think it was supposed to decode time information and know if GPS has valid time
}

/*************************** CRC functions (Firmware Reference Manual, p.32 + APN-030 Rev 1 Application Note) ***************************/

unsigned long ByteSwap (unsigned long n) 
{ 
   return ( ((n &0x000000FF)<<24) + ((n &0x0000FF00)<<8) + ((n &0x00FF0000)>>8) + (( n &0xFF000000)>>24) ); 
}

#define CRC32_POLYNOMIAL    0xEDB88320L

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
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
unsigned long CalculateBlockCRC32
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

void gps_print_raw(unsigned char gps_data[], int size)
{
    int i;

    printf("GPS: ");

    for(i = 0; i < size; i++)
    {           
        printf("%02X ", gps_data[i]);
    }
    
    printf("\n");
}

void gps_print_formatted(gps_t* data)
{
    printf("GPS: p(%.3lf %.3lf %.3lf) sp(%.3lf %.3lf %.3lf) v(%.3lf %.3lf %.3lf) sv(%.3lf %.3lf %.3lf) status(%lx %lx)\n",
            data->p[0], data->p[1], data->p[2], data->sigma_p[0], data->sigma_p[1], data->sigma_p[2], data->v[0], data->v[1], data->v[2], data->sigma_v[0], data->sigma_v[1], data->sigma_v[2], (long)data->p_status, (long)data->v_status);
}
