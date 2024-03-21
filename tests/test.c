/*  CANFix - An Open Source CANBus based Flight Information Protocol
 *  Copyright (c) 2021 Phil Birkelbach
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  This file contains the source code for the CANBus interface functions
 */

#include <pthread.h>
#include <signal.h>

#include "can.h"
#include "canfix.h"

static pthread_t _thread;
static int _quitflag;
static uint8_t _tc_config[8];

static void *
_msg_thread(void *x) {
    struct can_frame frame;
    int result;

    while(!_quitflag) {
        result = can_read(&frame);
        if(result > 0) {
            canfix_exec(frame.can_id, frame.can_dlc, frame.data);
        }
    }
}

static void
_quit_signal(int sig)
{
    _quitflag = 1;
}

/* Callback function to the canfix library for sending a frame.  We
 * translate between the two structures and then call the can_write function */
static int
_write_frame(uint16_t id, uint8_t dlc, uint8_t *data) {
    struct can_frame sframe;

    sframe.can_id = id;
    sframe.can_dlc = dlc;
    memcpy(sframe.data, data, 8);
    return can_write(sframe);
}

static uint8_t
_config_query_callback(uint16_t key, uint8_t *data, uint8_t *length) {
    if(key < 8) {
            data[0] = _tc_config[key];
            *length = 1;
            return 0;
    }
}

static uint8_t
_config_set_callback(uint16_t key, uint8_t *data, uint8_t length) {
    if(key < 8) {
        _tc_config[key] = data[0];
        return 0;
    }
}

static void
_node_set_callback(uint8_t node) {
    printf("New Node = %d\n", node);
}

static void
_alarm_callback(uint8_t node, uint16_t code, uint8_t *data, uint8_t length) {
    printf("Received Alarm Code 0x%X from node0x%X\n", code, node);
}

static void
_parameter_callback(canfix_parameter par) {
    printf("Received Parameter %d from Node 0x%X\n", par.type, par.node);
}

int
main(int argc, const char *argv[]) {
    struct sigaction sa;
    //struct can_frame frame;
    canfix_parameter par;
    int32_t altitude = -1000;
    int ch;

    /* Set up the signal handlers */
    memset (&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = &_quit_signal;
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);


    can_setup("vcan0");
    canfix_init(0x23, 0x23, 1, 0x770001);
    canfix_set_write_callback(_write_frame);
    canfix_set_node_set_callback(_node_set_callback);
    canfix_set_alarm_callback(_alarm_callback);
    canfix_set_parameter_callback(_parameter_callback);
    canfix_set_query_callback(_config_query_callback);
    canfix_set_config_callback(_config_set_callback);

    pthread_create(&_thread, NULL, _msg_thread, NULL);

    par.type = 0x184;
    par.index = 0;
    par.flags = 0;
    par.meta = 0;
    par.length = 4;
    while(!_quitflag) {
        memcpy(par.data, &altitude, sizeof(altitude));
        altitude += 10;
        canfix_send_parameter(par);
        usleep(100000);
    }
    pthread_join(_thread, NULL);
    can_close();
    exit(0);
}
