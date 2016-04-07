/*
 * Copyright 漏 2001-2010 St茅phane Raimbault <stephane.raimbault@gmail.com>
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

/*
   The library is designed to send and receive data from a device that
   communicate via the Modbus protocol.

   The function names used are inspired by the Modicon Modbus Protocol
   Reference Guide which can be obtained from Schneider at
   www.schneiderautomation.com.

   Documentation:
   http://www.easysw.com/~mike/serial/serial.html
   http://copyleft.free.fr/wordpress/index.php/libmodbus/
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#include <termios.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>

/* TCP */
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#if defined(__FreeBSD__ ) && __FreeBSD__ < 5
#include <netinet/in_systm.h>
#endif
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#if !defined(UINT16_MAX)
#define UINT16_MAX 0xFFFF
#endif

#include "serial.h"

#define UNKNOWN_ERROR_MSG "Not defined in modbus specification"

#define DEBUG

#ifdef DEBUG
#define wprintf(format, arg...)  \
    printf( format , ##arg)
#else
#define wprintf(format, arg...)
#endif


/* Table of CRC values for high-order byte */
static uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

c_serial::c_serial (const char *device,
                    int baud, const char *parity, int data_bit,
                    int stop_bit) {
    mb_param = new serial_param_t;

    init_rtu (device, baud, parity, data_bit, stop_bit);
}

c_serial::~c_serial () {
    delete mb_param;
}

void c_serial::sleep (long int s, long int us) {
    struct timeval tv;
    tv.tv_sec = s;
    tv.tv_usec = us;
    if (select (0, NULL, NULL, NULL, &tv) < 0) {
        perror ("[serial_sleep: select error]");
    }
}

/* Treats errors and flush or close connection if necessary */
void c_serial::error_treat (int code, const char *string) {
    /*fprintf ( stderr, "\nerror_treat: ERROR %s (%0X)\n", string, -code );*/
    if (mb_param->debug) {
        wprintf ("\033[31;40;1m \nerror_treat: %s (%0X)\n\033[0m", string, -code);
    }

    if (mb_param->error_handling == FLUSH_OR_CONNECT_ON_ERROR) {
        switch (code) {
        case INVALID_DATA:
        case INVALID_CRC:
        case INVALID_EXCEPTION_CODE:
            flush ();
            break;
        case SELECT_FAILURE:
        case SOCKET_FAILURE:
        case CONNECTION_CLOSED:
            close ();
            connect ();
            break;
        default:
            /* NOP */
            break;
        } // end switch
    } // end if
}

void c_serial::flush () {
    if (mb_param->type_com == RTU) {
        tcflush (mb_param->fd, TCIOFLUSH);
    } else {
        int ret;
        do {
            /* Extract the garbage from the socket */
            char devnull[MAX_ADU_LENGTH_TCP];
#if (!HAVE_DECL___CYGWIN__)
            ret = recv (mb_param->fd, devnull, MAX_ADU_LENGTH_TCP, MSG_DONTWAIT);
#else
            /* On Cygwin, it's a bit more complicated to not wait */
            fd_set rfds;
            struct timeval tv;

            tv.tv_sec = 0;
            tv.tv_usec = 0;
            FD_ZERO (&rfds);
            FD_SET (mb_param->fd, &rfds);
            ret = select (mb_param->fd + 1, &rfds, NULL, NULL, &tv);
            if (ret > 0) {
                ret = recv (mb_param->fd, devnull, MAX_ADU_LENGTH_TCP, 0);
            } else if (ret == -1) {
                /* error_treat() doesn't call serial_flush() in
                this case (avoid infinite loop) */
                error_treat (SELECT_FAILURE, "Select failure");
            }
#endif
            if (mb_param->debug && ret > 0) {
                wprintf ("%d bytes flushed\n", ret);
            }
        } while (ret > 0);
    }
}

/* Sends a query/response over a serial or a TCP communication */
int c_serial::send (uint8_t *query, int query_length) {
    int i;
    int ret;
    uint16_t s_crc;

    if (mb_param->debug) {
        wprintf ("\033[34;40;1m \nsend:\033[0m");
        printf ("%s", query);
    }

    if (mb_param->type_com == RTU) {
        /*tcflush ( mb_param->fd, TCIOFLUSH ); */
        ret = write (mb_param->fd, query, query_length);
    } else {
        ret = ::send (mb_param->fd, query, query_length, MSG_NOSIGNAL);
    }

    /* Return the number of bytes written (0 to n)
    or SOCKET_FAILURE on error */
    if ( (ret == -1) || (ret != query_length)) {
        ret = SOCKET_FAILURE;
        error_treat (ret, "send: Write socket failure");
    }

    return ret;
}

#define WAIT_DATA()                                                                \
{                                                                                  \
    while ((select_ret = select(mb_param->fd+1, &rfds, NULL, NULL, &tv)) == -1) {  \
            if (errno == EINTR) {                                                  \
                    fprintf(stderr, "A non blocked signal was caught\n");          \
                    /* Necessary after an error */                                 \
                    FD_ZERO(&rfds);                                                \
                    FD_SET(mb_param->fd, &rfds);                                   \
            } else {                                                               \
                    error_treat(SELECT_FAILURE, "Select failure");       \
                    return SELECT_FAILURE;                                         \
            }                                                                      \
    }                                                                              \
                                                                                   \
    if (select_ret == 0) {                                                         \
            /* Timeout */                                                          \
            /* Call to error_treat is done later to manage exceptions */           \
            if ( mb_param->debug )                                                 \
                wprintf ( "\n" );                                                  \
            return SELECT_TIMEOUT;                                                 \
    }                                                                              \
}

int c_serial::read (uint8_t *msg, int select_time, int wait_time) {
    int select_ret;
    int read_ret;
    fd_set rfds;
    struct timeval tv;
    int msg_length;
    int length_to_read = 1024;
    uint8_t *p_msg;
    enum { FUNCTION, BYTE, COMPLETE };

    if (mb_param->debug) {
        wprintf ("\nWaiting for a message...\n");
    }

    /* Add a file descriptor to the set */
    FD_ZERO (&rfds);
    FD_SET (mb_param->fd, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = select_time * 1000;
    WAIT_DATA();
    sleep (0, wait_time * 1000);

    p_msg = msg;
    if (mb_param->debug) {
        wprintf ("\033[32;40;1m \nrcv:\033[0m");
    }
    if (mb_param->type_com == RTU) {
        read_ret = ::read (mb_param->fd, p_msg, length_to_read);
    } else {
        read_ret = recv (mb_param->fd, p_msg, length_to_read, 0);
    }
    if (read_ret == 0) {
        return CONNECTION_CLOSED;
    } else if (read_ret < 0) {
        /* The only negative possible value is -1 */
        error_treat (SOCKET_FAILURE, "rcv_msg: Read socket failure");
        return SOCKET_FAILURE;
    }

    /* Display the hex code of each character received */
    if (mb_param->debug) {
        printf ("%s\n", p_msg);
    }

    return 0;
}

/* Initializes the serial_param_t structure for RTU
   - device: "/dev/ttyS0"
   - baud:   9600, 19200, 57600, 115200, etc
   - parity: "even", "odd" or "none"
   - data_bits: 5, 6, 7, 8
   - stop_bits: 1, 2
*/
void c_serial::init_rtu (const char *device,
                                int baud, const char *parity, int data_bit,
                                int stop_bit) {
    memset (mb_param, 0, sizeof (serial_param_t));
    strcpy (mb_param->device, device);
    mb_param->baud = baud;
    strcpy (mb_param->parity, parity);
    mb_param->debug = FALSE;
    mb_param->data_bit = data_bit;
    mb_param->stop_bit = stop_bit;
    mb_param->type_com = RTU;
    mb_param->error_handling = FLUSH_OR_CONNECT_ON_ERROR;
}

/* Sets up a serial port for RTU communications */
int c_serial::connect () {
    struct termios tios;
    speed_t speed;

    if (mb_param->debug) {
        fprintf (stderr, "Opening %s at %d bauds (%s)\n", mb_param->device, mb_param->baud, mb_param->parity);
    }

    /* The O_NOCTTY flag tells UNIX that this program doesn't want
       to be the "controlling terminal" for that port. If you
       don't specify this then any input (such as keyboard abort
       signals and so forth) will affect your process

       Timeouts are ignored in canonical input mode or when the
       NDELAY option is set on the file via open or fcntl */
    mb_param->fd = open (mb_param->device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);
    if (mb_param->fd < 0) {
        fprintf (stderr, "ERROR Can't open the device %s (%s)\n", mb_param->device, strerror (errno));
        return -1;
    }

    /* Save */
    tcgetattr (mb_param->fd, & (mb_param->old_tios));

    memset (&tios, 0, sizeof (struct termios));

    /* C_ISPEED     Input baud (new interface)
       C_OSPEED     Output baud (new interface)
    */
    switch (mb_param->baud) {
    case 110:
        speed = B110;
        break;
    case 300:
        speed = B300;
        break;
    case 600:
        speed = B600;
        break;
    case 1200:
        speed = B1200;
        break;
    case 2400:
        speed = B2400;
        break;
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    default:
        speed = B9600;
        fprintf (stderr, "WARNING Unknown baud rate %d for %s (B9600 used)\n", mb_param->baud, mb_param->device);
    }

    /* Set the baud rate */
    if ( (cfsetispeed (&tios, speed) < 0) || (cfsetospeed (&tios, speed) < 0)) {
        perror ("cfsetispeed/cfsetospeed\n");
        return -1;
    }

    /* C_CFLAG      Control options
       CLOCAL       Local line - do not change "owner" of port
       CREAD        Enable receiver
    */
    tios.c_cflag |= (CREAD | CLOCAL);
    /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

    /* Set data bits (5, 6, 7, 8 bits)
       CSIZE        Bit mask for data bits
    */
    tios.c_cflag &= ~CSIZE;
    switch (mb_param->data_bit) {
    case 5:
        tios.c_cflag |= CS5;
        break;
    case 6:
        tios.c_cflag |= CS6;
        break;
    case 7:
        tios.c_cflag |= CS7;
        break;
    case 8:
    default:
        tios.c_cflag |= CS8;
        break;
    }

    /* Stop bit (1 or 2) */
    if (mb_param->stop_bit == 1)
        tios.c_cflag &= ~ CSTOPB;
    else /* 2 */
        tios.c_cflag |= CSTOPB;

    /* PARENB       Enable parity bit
       PARODD       Use odd parity instead of even */
    if (strncmp (mb_param->parity, "none", 4) == 0) {
        tios.c_cflag &= ~ PARENB;
    } else if (strncmp (mb_param->parity, "even", 4) == 0) {
        tios.c_cflag |= PARENB;
        tios.c_cflag &= ~ PARODD;
    } else {
        /* odd */
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
    }

    /* Read the man page of termios if you need more information. */

    /* This field isn't used on POSIX systems
       tios.c_line = 0;
    */

    /* C_LFLAG      Line options

       ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
       ICANON       Enable canonical input (else raw)
       XCASE        Map uppercase \lowercase (obsolete)
       ECHO Enable echoing of input characters
       ECHOE        Echo erase character as BS-SP-BS
       ECHOK        Echo NL after kill character
       ECHONL       Echo NL
       NOFLSH       Disable flushing of input buffers after
       interrupt or quit characters
       IEXTEN       Enable extended functions
       ECHOCTL      Echo control characters as ^char and delete as ~?
       ECHOPRT      Echo erased character as character erased
       ECHOKE       BS-SP-BS entire line on line kill
       FLUSHO       Output being flushed
       PENDIN       Retype pending input at next read or input char
       TOSTOP       Send SIGTTOU for background output

       Canonical input is line-oriented. Input characters are put
       into a buffer which can be edited interactively by the user
       until a CR (carriage return) or LF (line feed) character is
       received.

       Raw input is unprocessed. Input characters are passed
       through exactly as they are received, when they are
       received. Generally you'll deselect the ICANON, ECHO,
       ECHOE, and ISIG options when using raw input
    */

    /* Raw input */
    tios.c_lflag &= ~ (ICANON | ECHO | ECHOE | ISIG);

    /* C_IFLAG      Input options

       Constant     Description
       INPCK        Enable parity check
       IGNPAR       Ignore parity errors
       PARMRK       Mark parity errors
       ISTRIP       Strip parity bits
       IXON Enable software flow control (outgoing)
       IXOFF        Enable software flow control (incoming)
       IXANY        Allow any character to start flow again
       IGNBRK       Ignore break condition
       BRKINT       Send a SIGINT when a break condition is detected
       INLCR        Map NL to CR
       IGNCR        Ignore CR
       ICRNL        Map CR to NL
       IUCLC        Map uppercase to lowercase
       IMAXBEL      Echo BEL on input line too long
    */
    if (strncmp (mb_param->parity, "none", 4) == 0) {
        tios.c_iflag &= ~INPCK;
    } else {
        tios.c_iflag |= INPCK;
    }

    /* Software flow control is disabled */
    tios.c_iflag &= ~ (IXON | IXOFF | IXANY);

    /* C_OFLAG      Output options
       OPOST        Postprocess output (not set = raw output)
       ONLCR        Map NL to CR-NL

       ONCLR ant others needs OPOST to be enabled
    */

    /* Raw ouput */
    tios.c_oflag &= ~ OPOST;

    /* C_CC         Control characters
       VMIN         Minimum number of characters to read
       VTIME        Time to wait for data (tenths of seconds)

       UNIX serial interface drivers provide the ability to
       specify character and packet timeouts. Two elements of the
       c_cc array are used for timeouts: VMIN and VTIME. Timeouts
       are ignored in canonical input mode or when the NDELAY
       option is set on the file via open or fcntl.

       VMIN specifies the minimum number of characters to read. If
       it is set to 0, then the VTIME value specifies the time to
       wait for every character read. Note that this does not mean
       that a read call for N bytes will wait for N characters to
       come in. Rather, the timeout will apply to the first
       character and the read call will return the number of
       characters immediately available (up to the number you
       request).

       If VMIN is non-zero, VTIME specifies the time to wait for
       the first character read. If a character is read within the
       time given, any read will block (wait) until all VMIN
       characters are read. That is, once the first character is
       read, the serial interface driver expects to receive an
       entire packet of characters (VMIN bytes total). If no
       character is read within the time allowed, then the call to
       read returns 0. This method allows you to tell the serial
       driver you need exactly N bytes and any read call will
       return 0 or N bytes. However, the timeout only applies to
       the first character read, so if for some reason the driver
       misses one character inside the N byte packet then the read
       call could block forever waiting for additional input
       characters.

       VTIME specifies the amount of time to wait for incoming
       characters in tenths of seconds. If VTIME is set to 0 (the
       default), reads will block (wait) indefinitely unless the
       NDELAY option is set on the port with open or fcntl.
    */
    /* Unused because we use open with the NDELAY option */
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    /* 处理未接收字符 */
    tcflush (mb_param->fd, TCIFLUSH);

    if (tcsetattr (mb_param->fd, TCSANOW, &tios) < 0) {
        perror ("tcsetattr\n");
        return -1;
    }

    return 0;
}

/* Closes a modbus connection */
void c_serial::close () {
    if (tcsetattr (mb_param->fd, TCSANOW, & (mb_param->old_tios)) < 0)
        perror ("tcsetattr");
    ::close (mb_param->fd);
}

/* Activates the debug messages */
void c_serial::set_debug (int boolean) {
    mb_param->debug = boolean;
}
