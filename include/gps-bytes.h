/**
 * @file      gps-bytes.h
 * @author    George Andrew Brindeiro
 * @date      06/10/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

// Binary Message Format = 3 sync + 25 header + variable data + 4 CRC

/* Binary Header */

// Header byte order/format
#define SYNC0           0   // char
#define SYNC1           1   // char
#define SYNC2           2   // char
#define HDR_LEN         3   // uchar
#define MSG_ID          4   // ushort
#define MSG_TYPE        6   // char
#define PORT_ADDR       7   // uchar
#define MSG_LEN         8   // ushort
#define SEQ_NUM         10  // ushort
#define IDLE_T          12  // uchar
#define T_STATUS        13  // enum
#define T_WEEK          14  // ushort
#define T_MS            16  // GPSec (ulong)
#define GPS_STATUS      20  // ulong
#define RESERVED        24  // ushort
#define SW_VERS         26  // ushort
#define DATA            28  // variable

// Default values
#define D_SYNC0         0xAA
#define D_SYNC1         0x44
#define D_SYNC2         0x12
#define D_HDR_LEN       28

// Multi-byte sizes
#define S_MSG_ID        2
#define S_MSG_LEN       2
#define S_SEQ_NUM       2
#define S_T_WEEK        2
#define S_T_MS          4
#define S_GPS_STATUS    4
#define S_RESERVED      2
#define S_SW_VERS       2
#define S_CRC           4

// Message types
#define BINARY          0x00
#define ASCII           0x10
#define ABR_ASCII       0x20
#define NMEA            0x20        // Not a bug, Abbreviated ASCII = NMEA
#define RESPONSE        0x80        // Bit 7 defines if packet is original message (0) or response (1)

// Port addresses in gps-ids.h
// Time status in gps-status.h
// GPS status in gps-status.h

/* BESTXYZ */

#define BXYZ_PSTAT      (D_HDR_LEN)
#define BXYZ_PTYPE      (D_HDR_LEN+4)
#define BXYZ_PX         (D_HDR_LEN+8)
#define BXYZ_PY         (D_HDR_LEN+16)
#define BXYZ_PZ         (D_HDR_LEN+24)
#define BXYZ_sPX        (D_HDR_LEN+32)
#define BXYZ_sPY        (D_HDR_LEN+36)
#define BXYZ_sPZ        (D_HDR_LEN+40)
#define BXYZ_VSTAT      (D_HDR_LEN+44)
#define BXYZ_VTYPE      (D_HDR_LEN+48)
#define BXYZ_VX         (D_HDR_LEN+52)
#define BXYZ_VY         (D_HDR_LEN+60)
#define BXYZ_VZ         (D_HDR_LEN+68)
#define BXYZ_sVX        (D_HDR_LEN+76)
#define BXYZ_sVY        (D_HDR_LEN+80)
#define BXYZ_sVZ        (D_HDR_LEN+84)
#define BXYZ_STNID      (D_HDR_LEN+88)
#define BXYZ_VLATE      (D_HDR_LEN+92)
#define BXYZ_DIFFAGE    (D_HDR_LEN+96)
#define BXYZ_SOLAGE     (D_HDR_LEN+92)
#define BXYZ_SV         (D_HDR_LEN+92)
#define BXYZ_SOLSV      (D_HDR_LEN+92)
