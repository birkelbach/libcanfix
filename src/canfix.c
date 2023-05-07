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

#include <stdlib.h>
#include <stdio.h>

#include "canfix.h"

static uint8_t _node;
static uint8_t _device;
static uint8_t _revision;
static uint32_t _model;

static int (*_write_callback)(uint16_t, uint8_t, uint8_t *);
static uint8_t (*_read_byte_callback)(uint8_t);
static void (*_write_byte_callback)(uint8_t, uint8_t);
static void (*_node_set_callback)(uint8_t);
static void (*_bitrate_callback)(uint8_t);
static void (*_report_callback)(void);
static uint8_t (*_twoway_callback)(uint8_t, uint16_t);
static uint8_t (*_config_callback)(uint16_t, uint8_t *, uint8_t);
static uint8_t (*_query_callback)(uint16_t, uint8_t *, uint8_t *);
static void (*_parameter_callback)(canfix_parameter);
static void (*_alarm_callback)(uint8_t, uint16_t, uint8_t*, uint8_t);
//static void (*_stream_callback)(uint8_t, uint8_t *, uint8_t);



void
canfix_init(uint8_t node, uint8_t device, uint8_t revision, uint32_t model) {
	_node = node;
	_device = device;
	_revision = revision;
	_model = model;
}


void
canfix_set_write_callback(int (*f)(uint16_t, uint8_t, uint8_t *)) {
	_write_callback = f;
}

void
canfix_set_read_byte_callback(uint8_t (*f)(uint8_t)) {
	_read_byte_callback = f;
}

void
canfix_set_write_byte_callback(void (*f)(uint8_t, uint8_t)) {
	_write_byte_callback = f;
}


void
canfix_set_alarm_callback(void (*f)(uint8_t, uint16_t, uint8_t*, uint8_t)) {
    _alarm_callback = f;
}

void
canfix_set_parameter_callback(void (*f)(canfix_parameter)) {
    _parameter_callback = f;
}

void
canfix_set_node_set_callback(void (*f)(uint8_t)) {
    _node_set_callback = f;
}

void
canfix_set_report_callback(void (*f)(void)) {
    _report_callback = f;
}

void
canfix_set_twoway_callback(uint8_t (*f)(uint8_t, uint16_t)) {
    _twoway_callback = f;
}

void
canfix_set_config_callback(uint8_t (*f)(uint16_t, uint8_t *, uint8_t)) {
    _config_callback = f;
}

void
canfix_set_query_callback(uint8_t (*f)(uint16_t, uint8_t *, uint8_t *)) {
    _query_callback = f;
}

//void canfix_set_stream_callback(void (*f)(uint8_t, uint8_t *, uint8_t));


static void
_handle_node_specific(uint16_t id, uint8_t length, uint8_t *data) {
    uint8_t rlength;
    uint8_t rdata[8];

    // This prepares a generic response
    rdata[0] = data[0];
    rdata[1] = id - NSM_START;

    switch(data[0]) {
        case NSM_ID: // Node Identify Message
            if(data[1] == _node || data[1]==0) {
                rdata[2] = 0x01;
                rdata[3] = _device;
                rdata[4] = _revision;
                rdata[5] = (uint8_t)_model;
                rdata[6] = (uint8_t)(_model >> 8);
                rdata[7] = (uint8_t)(_model >> 16);
                rlength = 8;
                break;
            } else {
                return;
            }
            break;
        case NSM_BITRATE:
            if(data[1] == _node || data[0]==0) {
                if(data[2] >= 1 && data[2] <=4) {
					if(_bitrate_callback) {
						_bitrate_callback(data[2]);
				    }
                } else {
                    rdata[2] = 0x01;
                    rlength = 3;
                    _write_callback(NSM_START + _node, rlength, rdata);
                    return;
                }

            } else {
                return;
            }
            break;
        case NSM_NODE_SET: // Node Set Message
            if(data[1] == _node || data[0]==0) {
                if(data[2] != 0x00) { // Doesn't respond to broadcast
                    _node = data[2];
                    if(_node_set_callback) {
                        _node_set_callback(_node);
                    }
                    rdata[2] = 0x00;
                } else {
                    rdata[2] = 0x01;
                }
                rlength = 2;
            } else {
                return;
            }
            break;
        /* We use a bitmask in the EEPROM to determine whether or not a
           parameter is disabled.  A 0 in the bit location means enabled
           and a 1 means disabled. */
        case NSM_DISABLE:
            if(data[1] == _node || data[1]==0) {
                ; //parameterEnable(frame);
            }
            return;
        case NSM_ENABLE:
            if(data[1] == _node) {
                ; //parameterEnable(frame);
            }
            return;
        case NSM_REPORT:
            if(data[1] == _node || data[1]==0) {
                if(_report_callback) _report_callback();
            }
            return;
        case NSM_FIRMWARE: //Not implemented yet
            if(data[1] == _node) {
                ; // Do some stuff
            }
            return;
        case NSM_TWOWAY:
            if(data[1] == _node) {
                if(_twoway_callback && data[0]!=0x00) {
                    if(_twoway_callback(data[2], *((uint16_t *)(&data[3]))) == 0) {
                        rdata[2]=0x00;
                        rlength = 3;
                        break;
                    }
                }
            } else {
                return;
            }
        case NSM_CONFSET:
            if(data[1] == _node) {
                if(_config_callback) {
                    rdata[2] = _config_callback(*((uint16_t *)(&data[2])), (uint8_t *)&data[4], length);
                } else {
                    rdata[2] = 1;
                }
                rlength = 3;
                break;
            } else {
                return;
            }
        case NSM_CONFGET:
            if(data[1] == _node) {
                if(_query_callback) {
                    rdata[2] = _query_callback(*((uint16_t *)(&data[2])), &rdata[3], &length);
                } else {
                    rdata[2] = 1;
                }
                if(rdata[2]==0) { /* Send Success with data */
                    rlength = 3+length;
                } else {
                    rlength = 3; /* Just send the error */
                }
                break;
            } else {
                return;
            }
        default:
            return;
    }
    _write_callback(NSM_START + _node, rlength, rdata);
}

void
canfix_exec(uint16_t id, uint8_t length, uint8_t *data) {
    uint8_t n;
    canfix_parameter par;

    if(id == 0x00) { /* Ignore ID 0 */
        ;
    } else if(id < 256) { /* Node Alarms */
        if(_alarm_callback) {
            _alarm_callback(id, *((uint16_t *)(&data[0])), &data[2], length-2);
        }
    } else if(id < 0x6E0) { /* Parameters */
        if(_parameter_callback) {
            par.type = id;
            par.node = data[0];
            par.index = data[1];
            par.meta = data[2] >> 4;
            par.flags = data[2] & 0x0F;
            par.length = length - 3;
            for(n = 0; n<par.length; n++) par.data[n] = data[3+n];
            _parameter_callback(par);
        }
    } else if(id < 0x7E0) { /* Node Specific Message */
        _handle_node_specific(id, length, data);
    } else { /* Communication Channel */
        ; /* Not implemented at the moment */
    }
}

int
canfix_send_parameter(canfix_parameter par) {
    /* TODO: Do some bounds checking */
    uint8_t data[8];

    data[0] = _node;
    data[1] = par.index;
    data[2] = par.flags | (par.meta << 4);
    for(uint8_t n=0; n<5; n++) data[3+n] = par.data[n];
    _write_callback(par.type, par.length+3, data);
	return 0;
}

void
canfix_send_identification(uint8_t dest) {
    uint8_t data[8];

    data[0] = 0;
    data[1] = dest;
    data[2] = 1;
    data[3] = _device;
    data[4] = _revision;
    memcpy(&data[5], &_model, 3);

    _write_callback(_node + 0x6E0, 8, data);
}

int
canfix_send_node_status(uint16_t ptype, uint8_t *data, uint8_t len) {
	uint8_t buff[8];

	if(len < 1 || len > 5) {
		return -1;
	}

	buff[0] = 0x06;
	buff[1] = ptype;
	buff[2] = ptype >> 8;
	for(int n=0;n<len;n++) {
		buff[3+n] = data[n];
	}
	return _write_callback(_node + 0x6E0, len+3, buff);
}
