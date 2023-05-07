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



#include "can.h"

static int _can_socket;
static unsigned int _rx_counter;
static unsigned int _tx_counter;


int
can_setup(const char *device) {
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct timeval tv;

    _can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(_can_socket < 0) {
        perror("Socket");
        return 1;
    }
    if(device == NULL)
        strcpy(ifr.ifr_name, "vcan0");
    else
        strcpy(ifr.ifr_name, device);
    ioctl(_can_socket, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(_can_socket, (struct sockaddr *)&addr, sizeof(addr))) {
        perror("Bind");
        return 1;
    }

    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(_can_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
}


int
can_write(struct can_frame frame) {
    if(write(_can_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        return -1;
    } else {
        _tx_counter++;
    }
    return 0;
}

int
can_read(struct can_frame *frame) {
    int result;
    
    result = read(_can_socket, frame, sizeof(struct can_frame));
    if(result<0) {
        return result;
    } else {
        _rx_counter++;
    }
    return result;
}

void
can_close(void) {
    if(close(_can_socket) < 0) {
        perror("Close");
    }
}
