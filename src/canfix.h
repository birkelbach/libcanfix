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


#ifndef __CANFIX_H
#define __CANFIX_H

#include <stdint.h>
#include <string.h>

// Node Specific Message Control Codes
#define NSM_START    0x6E0
#define CH_START     0x7E0

#define NSM_ID       0
#define NSM_BITRATE  1
#define NSM_NODE_SET 2
#define NSM_DISABLE  3 // Disable Parameter
#define NSM_ENABLE   4 // Enable Parameter
#define NSM_REPORT   5
#define NSM_STATUS   6
#define NSM_FIRMWARE 7
#define NSM_TWOWAY   8
#define NSM_CONFSET  9  // Configuration Set
#define NSM_CONFGET  10 // Configuration Query
#define NSM_DESC     11 // Node description
#define NSM_PSET     12 //12 - 19 are the parameter set codes

#define NODESTAT_STATUS    0
#define NODESTAT_TEMP      1
#define NODESTAT_VOLT	   2
#define NODESTAT_CANTX     3
#define NODESTAT_CANRX     4
#define NODESTAT_CANTXERR  5
#define NODESTAT_CANRXERR  6


#define FCB_ANNUNC    0x01
#define FCB_QUALITY   0x02
#define FCB_FAIL      0x04


typedef struct _canfix_parameter {
	uint16_t type;
	uint8_t node;
	uint8_t index;
    uint8_t meta;
	uint8_t flags;
	uint8_t data[5];
	uint8_t length;
} canfix_parameter;


void canfix_init(uint8_t node, uint8_t device, uint8_t revision, uint32_t model);
void canfix_set_description(char *description);

void canfix_set_write_callback(int (*f)(uint16_t, uint8_t, uint8_t *));
/* These two are for bit fields that are used by the library for long term storage
 * of parameter enables */
void canfix_set_read_byte_callback(uint8_t (*f)(uint8_t));
void canfix_set_write_byte_callback(void (*f)(uint8_t, uint8_t));

void canfix_set_node_set_callback(void (*f)(uint8_t));
void canfix_set_alarm_callback(void (*f)(uint8_t, uint16_t, uint8_t*, uint8_t));
void canfix_set_parameter_callback(void (*f)(canfix_parameter));

void canfix_set_report_callback(void (*f)(void));
void canfix_set_twoway_callback(uint8_t (*f)(uint8_t, uint16_t));
void canfix_set_config_callback(uint8_t (*f)(uint16_t, uint8_t *, uint8_t));
void canfix_set_query_callback(uint8_t (*f)(uint16_t, uint8_t *, uint8_t *));
void canfix_set_firmware_callback(uint8_t (*f)(uint16_t, uint8_t));

//void canfix_set_stream_callback(void (*f)(uint8_t, uint8_t *, uint8_t));


void canfix_exec(uint16_t, uint8_t, uint8_t*);

int canfix_send_parameter(canfix_parameter par);
void canfix_send_identification(uint8_t dest);
int canfix_send_node_status(uint16_t ptype, uint8_t *data, uint8_t len);



#endif /* __CANFIX_H */
