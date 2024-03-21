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

/* This is a test / development node that pretends to be the MakerPlane
   12 input DI module */

#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <ncurses.h>

#include "config.h"
#include "../can.h"
#include "canfix.h"

#define MODE_NORMAL 0
#define MODE_FIRMWARE  1

#define STATE_WAITING 0
#define STATE_BLOCK 1

static int _quitflag;

static uint32_t _node_status;
static uint32_t _tx_counter;
static uint32_t _rx_counter;
static uint32_t _tx_error;
static uint32_t _rx_error;
static uint16_t _fwcode = 0xF23;
static uint8_t _fwchannel;
static int _mode; /* Set to 1 to start firmware loading*/
static pthread_mutex_t _write_lock;

extern switch_def_t inputs[12];
extern pid_def_t pids[PID_COUNT];


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
    int result;

    sframe.can_id = id;
    sframe.can_dlc = dlc;
    memcpy(sframe.data, data, 8);
    pthread_mutex_lock(&_write_lock);
    result = can_write(sframe);
    if(result) {
        _tx_error++;
    } else {
        _tx_counter++;
    }
    pthread_mutex_unlock(&_write_lock);
    return result;
}


static void
_node_set_callback(uint8_t node) {
    ; //printf("New Node = %d\n", node);
}

static void
_alarm_callback(uint8_t node, uint16_t code, uint8_t *data, uint8_t length) {
    ; //printf("Received Alarm Code 0x%X from node0x%X\n", code, node);
}

static void
_parameter_callback(canfix_parameter par) {
    ;
}


static void
_handle_firmware(uint8_t length, uint8_t *data, uint8_t reset) {
    static int state;
    static uint32_t address, offset;
    static uint8_t subsystem;
    static uint8_t blocksize;
    static uint8_t blocktype;
    uint8_t rdata[8];

    if(reset) {
        state = STATE_WAITING;
        return;
    }
    if(state == STATE_WAITING) {
        if(length == 7) { /* This should be a start of block frame */
            blocktype = data[0];
            subsystem = data[1];
            blocksize = data[2];
            address = *((uint32_t *)&data[3]);
            offset = 0;

            mvprintw(10,2,"Starting Block: type=%d, ss=%d, size=%d, addr=%u\n", blocktype, subsystem, blocksize, address);
            refresh();
            _write_frame(CH_START + _fwchannel * 2 + 1, length, data);
            state = STATE_BLOCK;
        } else if(length == 1) { /* Might be an abort or end frame */
            if(data[0] == 0xFD) { /* End of Transmission */
                mvprintw(10,2,"End of Transmission\n");
                refresh();
            } else if(data[0] == 0xFE) { /* Abort Transmission*/
                mvprintw(10,2,"Abort Transmission\n");
                refresh();
            }
            _write_frame(CH_START + _fwchannel * 2 + 1, length, data);
            _mode = MODE_NORMAL; /* Bail out of firmware */
        } else {
            mvprintw(10,2,"Unexpected length = %d\n", length);
            refresh();
        }
    } else { /* Block receive mode */
        if(length == 0) { /* End of block message*/
            mvprintw(10,2,"Received end of block\n");
            refresh();
            _write_frame(CH_START + _fwchannel * 2 + 1, 0, data);
            state = STATE_WAITING;  /* state = block receive*/
        } else {
            //printf("Received data at offset %u\n", offset);
            _write_frame(CH_START + _fwchannel * 2 + 1, 4, (uint8_t *)&offset);
            offset += length;
        }
    }
}

static uint8_t
_firmware_callback(uint16_t vcode, uint8_t channel) {
    mvprintw(10,2,"Start firmware load vcode = 0x%02X, channel = %d\n", vcode, channel);
    refresh();
    if(_fwcode != vcode) {
        return 0x01;
    }
    _fwchannel = channel;
    _mode = MODE_FIRMWARE;
    /* Reset the state of the firmware handler */
    _handle_firmware(0, NULL, 1);
    return 0x00;
}


static void *
_msg_thread(void *x) {
    double start_time = 0.0;
    double this_time;
    struct timespec ts;
    struct can_frame frame;
    int result;

    while(!_quitflag) {
        result = can_read(&frame);
        if(result > 0) {
            _rx_counter++;
            if(_mode == MODE_NORMAL) {
                canfix_exec(frame.can_id, frame.can_dlc, frame.data);
            } else if(_mode == MODE_FIRMWARE) {
                if(frame.can_id == CH_START + _fwchannel * 2) {
                    _handle_firmware(frame.can_dlc, frame.data, 0);
                } else {
                    // TODO do we want to do canfix_exec??? Probably not just ignore
                    ;
                }
                // TODO Remove this!!!!
                usleep(10000);
                // _write_frame(CH_START + _fwchannel * 2 + 1, frame.can_dlc, frame.data);
            }
        } else {
            if(_mode == MODE_FIRMWARE) {
                printf("Firmware Timeout\n");
                _mode = MODE_NORMAL;
            }
        }
        //canfix_send_parameter(par);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        this_time = ts.tv_sec + (double)ts.tv_nsec / 1E9;
        if(this_time - start_time > 2.0) {
            clock_gettime(CLOCK_MONOTONIC, &ts);
            start_time = ts.tv_sec + (double)ts.tv_nsec / 1E9;

            canfix_send_node_status(NODESTAT_STATUS, (uint8_t *)&_node_status, 2);
	    	canfix_send_node_status(NODESTAT_CANTX, (uint8_t *)&_tx_counter, 4);
		    canfix_send_node_status(NODESTAT_CANRX, (uint8_t *)&_rx_counter, 4);
		    canfix_send_node_status(NODESTAT_CANTXERR, (uint8_t *)&_tx_error, 4);
		    canfix_send_node_status(NODESTAT_CANRXERR, (uint8_t *)&_rx_error, 4);
        }
    }
}

static void
_send_switch_change(int sw, int val) {
    static canfix_parameter par;
    int bit;

    par.type = pids[inputs[sw].pid_index].pid;
    par.index = 0;
    par.flags = 0;
    par.meta = 0;
    par.length = pids[inputs[sw].pid_index].len;

    bit = inputs[sw].bit;

    if(val) {
        pids[inputs[sw].pid_index].data[bit/8] |= 0x01 << bit % 8;
    } else {
        pids[inputs[sw].pid_index].data[bit/8] &= ~(0x01 << bit % 8);
        //par.data[sw/8] &= ~(0x01 << sw % 8);
    }
    memcpy(par.data, pids[inputs[sw].pid_index].data, par.length);
    canfix_send_parameter(par);
}

int
main(int argc, const char *argv[]) {
    struct sigaction sa;
    struct can_frame frame;
    pthread_t thread;
    int result;
    int ch;
    static uint8_t switches[12];
    int keymap[] = {'q', 'w', 'e', 'r', 'a', 's', 'd', 'f', 'z' ,'x', 'c' ,'v'};

    /* Set up the signal handlers */
    memset (&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = &_quit_signal;
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    can_setup("vcan0");
    canfix_init(0x23, 0x23, 1, 0x770001);
    canfix_set_description("MakerPlane DI12U2 Digital Input Module v0.1 SN 0000001 Firmware v1.2.3.4-May 2023 GPL");
    canfix_set_write_callback(_write_frame);
    canfix_set_node_set_callback(_node_set_callback);
    canfix_set_alarm_callback(_alarm_callback);
    canfix_set_parameter_callback(_parameter_callback);
    canfix_set_query_callback(config_query_callback);
    canfix_set_config_callback(config_set_callback);
    canfix_set_firmware_callback(_firmware_callback);

    config_init();

    pthread_mutex_init(&_write_lock, NULL);
    pthread_create(&thread, NULL, _msg_thread, NULL);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    timeout(-1);

    for(int n=0;n<12;n++) {
        mvprintw(1, n+1, "%c", keymap[n]);
        mvprintw(2, n+1,"%x", switches[n]);
        refresh();
    }
    mvprintw(1, 16, "o=clear, ESC=Exit");

    while(!_quitflag) {
        ch = getch();
        for(int n=0; n<12; n++) {
            if(ch == keymap[n]) {
                switches[n] ^= 0x01;
                _send_switch_change(n, switches[n]);
            } else if(ch == 'o') {
                switches[n] = 0x00;
                _send_switch_change(n, switches[n]);
            }
        }
        for(int n=0;n<12;n++) {
            mvprintw(2, n+1,"%x", switches[n]);
            refresh();
        }
        if(ch==27) { /* ESC Key */
            _quitflag = 1;
        }
    }

    endwin();
    pthread_join(thread, NULL);
    can_close();
    config_debug();
    exit(0);
}
