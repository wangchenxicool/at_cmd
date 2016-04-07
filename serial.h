/*
 * Copyright © 2001-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser Public License for more details.
 *
 * You should have received a copy of the GNU Lesser Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MODBUS_H_
#define _MODBUS_H_

#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#if defined(__FreeBSD__ ) && __FreeBSD__ < 5
#include <netinet/in_systm.h>
#endif
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <termios.h>

/* DATA TYPE */
#define INT8        0
#define UINT8       1
#define INT16       2
#define UINT16      3
#define INT32       4
#define UINT32      5
#define INT64       6
#define UINT64      7
#define FLOAT       8
#define DOUBLE      9

/* Local */
#define INVALID_DATA            -0x10
#define INVALID_CRC             -0x11
#define INVALID_EXCEPTION_CODE  -0x12

#define SELECT_TIMEOUT          -0x13
#define SELECT_FAILURE          -0x14
#define SOCKET_FAILURE          -0x15
#define CONNECTION_CLOSED       -0x16
#define MB_EXCEPTION            -0x17

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5:
 *  - RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 *  - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes
 */
#define MAX_PDU_LENGTH            253
#define MAX_ADU_LENGTH_RTU        256
#define MAX_ADU_LENGTH_TCP        260

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif

#define SELECT_TIMEOUT          -0x13
#define SELECT_FAILURE          -0x14
#define SOCKET_FAILURE          -0x15
#define CONNECTION_CLOSED       -0x16
#define MB_EXCEPTION            -0x17

/* define data type */
#define DATA_TYPE_INT8          0
#define DATA_TYPE_UNINT8        1
#define DATA_TYPE_INT16         2
#define DATA_TYPE_UNINT16       3
#define DATA_TYPE_INT32         4
#define DATA_TYPE_UNINT32       5
#define DATA_TYPE_INT64         6
#define DATA_TYPE_UNINT64       7

typedef unsigned char  BYTE;
typedef unsigned long  DWORD;
typedef unsigned short WORD;
typedef void  VOID;
typedef bool  BOOL;

typedef enum {
    RTU = 0, TCP
}
type_com_t;

typedef enum { FLUSH_OR_CONNECT_ON_ERROR, NOP_ON_ERROR } error_handling_t;

/* This structure is byte-aligned */
typedef struct {
    /* Slave address */
    int slave;
    /* Descriptor (tty or socket) */
    int fd;
    /* Communication mode: RTU or TCP */
    type_com_t type_com;
    /* Flag debug */
    int debug;
    /* TCP port */
    int port;
    /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*"
       on Mac OS X for KeySpan USB<->Serial adapters this string
       had to be made bigger on OS X as the directory+file name
       was bigger than 19 bytes. Making it 67 bytes for now, but
       OS X does support 256 byte file names. May become a problem
       in the future. */
#ifdef __APPLE_CC__
    char device[64];
#else
    char device[16];
#endif
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    /* Data bit */
    uint8_t data_bit;
    /* Stop bit */
    uint8_t stop_bit;
    /* Parity: "even", "odd", "none" */
    char parity[5];
    /* In error_treat with TCP, do a reconnect or just dump the error */
    uint8_t error_handling;
    /* IP address */
    char ip[16];
    /* Save old termios settings */
    struct termios old_tios;
} serial_param_t;

typedef struct {
    int nb_coil_status;
    int nb_input_status;
    int nb_input_registers;
    int nb_holding_registers;
    uint8_t *tab_coil_status;
    uint8_t *tab_input_status;
    uint16_t *tab_input_registers;
    uint16_t *tab_holding_registers;

} serial_mapping_t;

class c_serial {
public:

    c_serial (const char *device,
              int baud, const char *parity, int data_bit,
              int stop_bit);
    ~c_serial ();

    /* All functions used for sending or receiving data return:
    - the numbers of values (bits or word) if success (0 or more)
    - less than 0 for exceptions errors
    */

    /* Initializes the serial_param_t structure for RTU.
       - device: "/dev/ttyS0"
       - baud:   9600, 19200, 57600, 115200, etc
       - parity: "even", "odd" or "none"
       - data_bits: 5, 6, 7, 8
       - stop_bits: 1, 2
    */
    void init_rtu (const char *device,
                          int baud, const char *parity, int data_bit,
                          int stop_bit);

    /* Establishes a modbus connexion.
       Returns 0 on success or -1 on failure. */
    int connect ();

    /* Closes a modbus connection */
    void close ();

    /* Activates the debug messages */
    void set_debug (int boolean);

    int read (uint8_t *msg, int select_time, int wait_time);

    int send (uint8_t *query, int query_length);

    void error_treat (int code, const char *string);

    void sleep (long int s, long int us);

private:
    serial_param_t *mb_param;
    /* Flush the pending request */
    void flush ();
};

#endif  /* _MODBUS_H_ */
