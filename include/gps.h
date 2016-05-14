/**
 * @file      gps.h
 * @author    George Andrew Brindeiro
 * @date      06/10/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

#include "gps-bytes.h"
#include "gps-ids.h"
#include "ros/ros.h"

// Max size of GPS data packets
#define GPS_PACKET_SIZE 200

/* GPS data struct */
typedef struct gps
{
    double status;
    double p_status;    // TO DO: implement gps_state, gps_p_status, v_status
    double p[3];
    double sigma_p[3];
    double v_status;
    double v[3];
    double sigma_v[3];
} gps_t;

/* Function Prototypes */

int gps_init(char* serial_port);
int gps_get_data(gps_t* data);
void gps_decode(gps_t* data, unsigned char gps_data[], unsigned short msg_id);
int gps_close();

void gps_configure(char* serial_port);
void gps_command(const char* command);
int gps_get_approx_time();
void gps_time(unsigned char t_status, unsigned short t_week, unsigned long t_ms);

unsigned long ByteSwap (unsigned long n);
unsigned long CRC32Value(int i);
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char *ucBuffer);

void gps_print_raw(unsigned char gps_data[], int size);
void gps_print_formatted(gps_t* data);
