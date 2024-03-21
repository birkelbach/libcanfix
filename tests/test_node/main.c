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

/* This is a test / development node that is used primarily to test the function
   of the CAN-FiX Utility program. */
#include <signal.h>
#include <time.h>

#include "../can.h"
#include "canfix.h"

#define MODE_NORMAL 0
#define MODE_FIRMWARE  1

#define STATE_WAITING 0
#define STATE_BLOCK 1

static int _quitflag;
static uint16_t _config[64];

static uint32_t _node_status;
static uint32_t _tx_counter;
static uint32_t _rx_counter;
static uint32_t _tx_error;
static uint32_t _rx_error;
static uint16_t _fwcode = 0xFFF;
static uint8_t _fwchannel;
static int _mode; /* Set to 1 to start firmware loading*/

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
    result = can_write(sframe);
    if(result) {
        _tx_error++;
    } else {
        _tx_counter++;
    }
    return result;
}

static uint8_t
_config_query_callback(uint16_t key, uint8_t *data, uint8_t *length) {
    if(key < 24) {
        data[0] = _config[key];
        data[1] = _config[key]>>8;
        *length = 2;
        return 0;
    } else {
        return 1;
    }
}

static uint8_t
_config_set_callback(uint16_t key, uint8_t *data, uint8_t length) {
    if(key < 8) {
        _config[key] = data[0] | ((uint16_t)data[1])<<8;
        printf("Received Configuration key %d, value %d\n", key, _config[key]);
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

            printf("Starting Block: type=%d, ss=%d, size=%d, addr=%u\n", blocktype, subsystem, blocksize, address);
            _write_frame(CH_START + _fwchannel * 2 + 1, length, data);
            state = STATE_BLOCK;
        } else if(length == 1) { /* Might be an abort or end frame */
            if(data[0] == 0xFD) { /* End of Transmission */
                printf("End of Transmission\n");
            } else if(data[0] == 0xFE) { /* Abort Transmission*/
                printf("Abort Transmission\n");
            }
            _write_frame(CH_START + _fwchannel * 2 + 1, length, data);
            _mode = MODE_NORMAL; /* Bail out of firmware */
        } else {
            printf("Unexpected length = %d\n", length);
        }
    } else { /* Block receive mode */
        if(length == 0) { /* End of block message*/
            printf("Received end of block\n");
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
    printf("Start firmware load vcode = 0x%02X, channel = %d\n", vcode, channel);
    if(_fwcode != vcode) {
        return 0x01;
    }
    _fwchannel = channel;
    _mode = MODE_FIRMWARE;
    /* Reset the state of the firmware handler */
    _handle_firmware(0, NULL, 1);
    return 0x00;
}


int
main(int argc, const char *argv[]) {
    struct sigaction sa;
    double start_time = 0.0;
    double this_time;
    struct timespec ts;
    struct can_frame frame;
    int result;

    /* Set up the signal handlers */
    memset (&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = &_quit_signal;
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    can_setup("vcan0");
    canfix_init(0xFE, 0xFE, 1, 0x000001);
    canfix_set_description("CAN-FiX Utility Test Node v0.1 SN 0000001 Firmware v1.2.3.4-May 2023 GPL");
    canfix_set_write_callback(_write_frame);
    canfix_set_node_set_callback(_node_set_callback);
    canfix_set_alarm_callback(_alarm_callback);
    canfix_set_parameter_callback(_parameter_callback);
    canfix_set_query_callback(_config_query_callback);
    canfix_set_config_callback(_config_set_callback);
    canfix_set_firmware_callback(_firmware_callback);

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
    can_close();
    exit(0);
}
