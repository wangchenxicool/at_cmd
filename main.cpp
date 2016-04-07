#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <sys/timeb.h>
#include "serial.h"

#define SLAVE         0x01

static int SPACE_TIME = 50;
static int STEP_MODE = 0;
static int COUNTS = 1;
static int WAIT_TIME = 0;

int main (int argc, char *argv[]) {
    int i, ret;
    uint8_t *rcv_buf;

    c_serial serial (argv[1], 9600, "none", 8, 1);
    serial.set_debug (TRUE);

    /* RTU parity : none, even, odd */
    if (serial.connect () == -1) {
        perror ("[serial_connect]");
        exit (1);
    }

    /* Allocate and initialize the different memory spaces */
    rcv_buf = (uint8_t *) malloc (1024 * sizeof (uint8_t));
    memset (rcv_buf, 0, 1024 * sizeof (uint8_t));

    serial.send ( (uint8_t*) argv[2], strlen (argv[2]));
    serial.send ( (uint8_t*) "\r\n", strlen ("\r\n"));
    sleep (1);
    ret = serial.read (rcv_buf, 5000, WAIT_TIME);
    if (ret < 0) {
        printf ("rcv err!\n");
    }

    serial.close ();
    free (rcv_buf);

    return 0;
}
