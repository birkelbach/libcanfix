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
#include <stdbool.h>
#include <stdio.h>

#include "canfix.h"

void
canfix_init(canfix_object *h, uint8_t node, uint8_t device, uint8_t revision, uint32_t model) {
    h->node = node;
    h->device = device;
    h->revision = revision;
    h->model = model;

    h->write_callback = NULL;
    h->alarm_callback = NULL;
    h->node_set_callback = NULL;
    h->bitrate_callback = NULL;
    h->report_callback = NULL;
    h->twoway_callback = NULL;
    h->config_callback = NULL;
    h->query_callback = NULL;
    h->parameter_callback = NULL;
    h->alarm_callback = NULL;
    h->firmware_callback = NULL;
}

/* Set's the node description string.  If this is set it will be sent after
   the Node Identification message is sent. */
void canfix_set_description(canfix_object *h, char *description) {
    h->description = description;
}

void
canfix_set_write_callback(canfix_object *h, int (*f)(uint16_t, uint8_t, uint8_t *)) {
	h->write_callback = f;
}

void
canfix_set_alarm_callback(canfix_object *h, void (*f)(uint8_t, uint16_t, uint8_t*, uint8_t)) {
    h->alarm_callback = f;
}

void
canfix_set_parameter_callback(canfix_object *h, void (*f)(canfix_parameter)) {
    h->parameter_callback = f;
}

void
canfix_set_node_set_callback(canfix_object *h, void (*f)(uint8_t)) {
    h->node_set_callback = f;
}

void
canfix_set_report_callback(canfix_object *h, void (*f)(void)) {
    h->report_callback = f;
}

void
canfix_set_twoway_callback(canfix_object *h, uint8_t (*f)(uint8_t, uint16_t)) {
    h->twoway_callback = f;
}

void
canfix_set_config_callback(canfix_object *h, uint8_t (*f)(uint16_t, uint8_t *, uint8_t)) {
    h->config_callback = f;
}

void
canfix_set_query_callback(canfix_object *h, uint8_t (*f)(uint16_t, uint8_t *, uint8_t *)) {
    h->query_callback = f;
}

void
canfix_set_firmware_callback(canfix_object *h, uint8_t (*f)(uint16_t, uint8_t)) {
    h->firmware_callback = f;
}

//void canfix_set_stream_callback(void (*f)(uint8_t, uint8_t *, uint8_t));


static void
_handle_node_specific(canfix_object *h, uint16_t id, uint8_t length, uint8_t *data) {
    uint8_t rlength;
    uint8_t rdata[8];

    // This prepares a generic response
    rdata[0] = data[0];
    rdata[1] = id - NSM_START;

    switch(data[0]) {
        case NSM_ID: // Node Identify Message
            if(data[1] == h->node || data[1]==0) {
                canfix_send_identification(h, id - NSM_START);
                return;
            } else {
                return;
            }
            break;
        case NSM_BITRATE:
            if(data[1] == h->node || data[0]==0) {
                if(data[2] >= 1 && data[2] <=4) {
                    if(h->bitrate_callback) {
                        h->bitrate_callback(data[2]);
                    }
                } else {
                    rdata[2] = 0x01;
                    rlength = 3;
                    h->write_callback(NSM_START + h->node, rlength, rdata);
                    return;
                }

            } else {
                return;
            }
            break;
        case NSM_NODE_SET: // Node Set Message
            if(data[1] == h->node || data[0]==0) {
                if(data[2] != 0x00) { // Doesn't respond to broadcast
                    h->node = data[2];
                    if(h->node_set_callback) {
                        h->node_set_callback(h->node);
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
            if(data[1] == h->node || data[1]==0) {
                ; //parameterEnable(frame);
            }
            return;
        case NSM_ENABLE:
            if(data[1] == h->node) {
                ; //parameterEnable(frame);
            }
            return;
        case NSM_REPORT:
            if(data[1] == h->node || data[1]==0) {
                if(h->report_callback) h->report_callback();
            }
            return;
        case NSM_FIRMWARE: //Not implemented yet
            if(data[1] == h->node) {
                if(h->firmware_callback) {
                    /* Pass verification code and channel request */
                    rdata[2] = h->firmware_callback(*((uint16_t *)(&data[2])), data[4]);
                    rlength = 3;
                    break;
                }
            }
            return;
        case NSM_TWOWAY:
            if(data[1] == h->node) {
                if(h->twoway_callback && data[0]!=0x00) {
                    if(h->twoway_callback(data[2], *((uint16_t *)(&data[3]))) == 0) {
                        rdata[2]=0x00;
                        rlength = 3;
                        break;
                    }
                }
            } else {
                return;
            }
        case NSM_CONFSET:
            if(data[1] == h->node) {
                if(h->config_callback) {
                    rdata[2] = h->config_callback(*((uint16_t *)(&data[2])), (uint8_t *)&data[4], length);
                } else {
                    rdata[2] = 1;
                }
                rlength = 3;
                break;
            } else {
                return;
            }
        case NSM_CONFGET:
            if(data[1] == h->node) {
                if(h->query_callback) {
                    rdata[2] = h->query_callback(*((uint16_t *)(&data[2])), &rdata[3], &length);
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
    h->write_callback(NSM_START + h->node, rlength, rdata);
}

void
canfix_exec(canfix_object *h, uint16_t id, uint8_t length, uint8_t *data) {
    uint8_t n;
    canfix_parameter par;

    if(id == 0x00) { /* Ignore ID 0 */
        ;
    } else if(id < 256) { /* Node Alarms */
        if(h->alarm_callback) {
            h->alarm_callback(id, *((uint16_t *)(&data[0])), &data[2], length-2);
        }
    } else if(id < 0x6E0) { /* Parameters */
        if(h->parameter_callback) {
            par.type = id;
            par.node = data[0];
            par.index = data[1];
            par.meta = data[2] >> 4;
            par.flags = data[2] & 0x0F;
            par.length = length - 3;
            for(n = 0; n<par.length; n++) par.data[n] = data[3+n];
            h->parameter_callback(par);
        }
    } else if(id < 0x7E0) { /* Node Specific Message */
        _handle_node_specific(h,id, length, data);
    } else { /* Communication Channel */
        ; /* Not implemented at the moment */
    }
}

int
canfix_send_parameter(canfix_object *h, canfix_parameter par) {
    /* TODO: Do some bounds checking */
    uint8_t data[8];

    data[0] = h->node;
    data[1] = par.index;
    data[2] = par.flags | (par.meta << 4);
    for(uint8_t n=0; n<5; n++) data[3+n] = par.data[n];
    h->write_callback(par.type, par.length+3, data);
	return 0;
}

void
canfix_send_identification(canfix_object *h, uint8_t dest) {
    uint8_t data[8];
    char c;
    int i, length;
    bool done;
    uint16_t packet;

    data[0] = NSM_ID;
    data[1] = dest;
    data[2] = 1;
    data[3] = h->device;
    data[4] = h->revision;
    memcpy(&data[5], &h->model, 3);
    h->write_callback(h->node + 0x6E0, 8, data);

    /* If we have a description string set then we'll send it here */
    if(h->description) {
        i = 0; packet = 0;
        done = false;
        data[0] = NSM_DESC;
        length = strlen(h->description);

        while(! done) {
            if(i<length) {
                c = h->description[i];
            } else {
                c = '\0';
            }
            data[i%4 + 4] = c;
            if(i%4 == 3) {
                data[2] = packet;
                data[3] = packet >> 8;
                packet++;
                h->write_callback(h->node + 0x6E0, 8, data);
                if(c == '\0') done = true;
            }
            i++;
        }
    }
}

int
canfix_send_node_status(canfix_object *h, uint16_t ptype, uint8_t *data, uint8_t len) {
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
	return h->write_callback(h->node + 0x6E0, len+3, buff);
}


#ifdef CANFIX_USE_QUEUE
/* If the queue feature is enabled then this function is used to push a message onto the queue
 * It will return CANFIX_QUEUE_OVERRUN if the queue is full in which case the oldest message on
 * the queue will be over written and lost.  Otherwise it returns zero.
 */
int
canfix_queue_push(canfix_object *h, uint16_t id, uint8_t length, uint8_t *data) {
    h->queue[h->head].id = id;
    h->queue[h->head].length = length;
    for(int n=0; n < length; n++) {
        h->queue[h->head].data[n] = data[n];
    }
    h->head++;
    if(h->head == CANFIX_QUEUE_LEN) h->head = 0;
    if(h->count == CANFIX_QUEUE_LEN) {
        h->tail++;
        if(h->tail == CANFIX_QUEUE_LEN) h->tail = 0;
        return CANFIX_QUEUE_OVERFLOW;
    }
    else h->count++;
    return 0;
}

/* If the queue feature is enabled then this function is used to execute the next message on the
 * queue.  It returns CANFIX_QUEUE_EMPTY if nothing was done because the queue was empty and zero
 * if canfix_exec() was called.
 */
int
canfix_queue_pop(canfix_object *h, uint16_t *id, uint8_t *length, uint8_t *data) {
    if(h->count == 0) return CANFIX_QUEUE_EMPTY;
    *id = h->queue[h->tail].id;
    *length = h->queue[h->tail].length;
    for(int n=0; n < *length; n++) {
        data[n] = h->queue[h->tail].data[n];
    }

    h->tail++;
    if(h->tail == CANFIX_QUEUE_LEN) h->tail = 0;
    h->count--;
    return 0;
}
#endif


