/**
 * @file      gps-ids.h
 * @author    George Andrew Brindeiro
 * @date      18/10/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

/* Log Message IDs */

#define BESTPOS			42
#define GPGGA			218
#define GPGSA			221 
#define GPRMC			225
#define BESTXYZ			241

/* Port Identifiers */

#define NO_PORTS		0x0000
#define COM1_ALL		0x0001
#define COM2_ALL		0x0002
#define COM3_ALL		0x0003
#define THISPORT_ALL	0x0006
#define ALL_PORTS		0x0008
#define XCOM1_ALL		0x0009
#define XCOM2_ALL		0x000A
#define USB1_ALL		0x000D
#define USB2_ALL		0x000E
#define USB3_ALL		0x000F
#define AUX_ALL			0x0010
#define XCOM3_ALL		0x0011
#define COM1			0x0020		// For COM1_N, with N=0:31, do (COM1 + N)
#define COM2			0x0040		// For COM2_N, with N=0:31, do (COM2 + N)
#define COM3			0x0060		// For COM3_N, with N=0:31, do (COM3 + N)
#define USB				0x0080		// For USB_N, with N=0:31, do (USB + N)
#define SPECIAL			0x00A0		// For SPECIAL_N, with N=0:31, do (SPECIAL + N)
#define THISPORT		0x00C0		// For THISPORT_N, with N=0:31, do (THISPORT + N)
#define FILEPORT		0x00E0		// For FILEPORT_N, with N=0:31, do (FILEPORT + N)
#define XCOM1			0x01A0		// For XCOM1_N, with N=0:31, do (XCOM1 + N)
#define XCOM2			0x02A0		// For XCOM2_N, with N=0:31, do (XCOM2 + N)
#define USB1			0x05A0		// For USB1_N, with N=0:31, do (USB1 + N)
#define USB2			0x06A0		// For USB2_N, with N=0:31, do (USB2 + N)
#define USB3			0x07A0		// For USB3_N, with N=0:31, do (USB3 + N)
#define AUX				0x08A0		// For AUX_N, with N=0:31, do (AUX + N)
#define XCOM3			0x09A0		// For XCOM3_N, with N=0:31, do (XCOM3 + N)
