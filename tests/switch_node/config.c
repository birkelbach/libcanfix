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

#include <ncurses.h>

#include "config.h"


switch_def_t inputs[12];
pid_def_t pids[PID_COUNT];
uint16_t update_rates[12];

void
config_init(void) {
    int n=0;

    pids[n].pid = 256;
    pids[n++].max = 7;
    pids[n].pid = 257;
    pids[n++].max = 7;
    pids[n].pid = 258;
    pids[n++].max = 13;
    pids[n].pid = 259;
    pids[n++].max = 13;
    pids[n].pid = 260;
    pids[n++].max = 7;
    pids[n].pid = 261;
    pids[n++].max = 7;
    pids[n].pid = 262;
    pids[n++].max = 7;
    pids[n].pid = 263;
    pids[n++].max = 7;
    pids[n].pid = 264;
    pids[n++].max = 7;
    pids[n].pid = 265;
    pids[n++].max = 7;
    pids[n].pid = 266;
    pids[n++].max = 7;
    pids[n].pid = 267;
    pids[n++].max = 7;
    pids[n].pid = 268;
    pids[n++].max = 15;
    pids[n].pid = 269;
    pids[n++].max = 15;
    pids[n].pid = 270;
    pids[n++].max = 3;
    pids[n].pid = 271;
    pids[n++].max = 3;
    pids[n].pid = 272;
    pids[n++].max = 2;
    pids[n].pid = 273;
    pids[n++].max = 2;
    pids[n].pid = 274;
    pids[n++].max = 5;
    pids[n].pid = 275;
    pids[n++].max = 5;
    pids[n].pid = 276;
    pids[n++].max = 7;
    pids[n].pid = 277;
    pids[n++].max = 7;
    pids[n].pid = 284;
    pids[n++].max = 39;
    pids[n].pid = 285;
    pids[n++].max = 39;
    pids[n].pid = 286;
    pids[n++].max = 39;
    pids[n].pid = 287;
    pids[n++].max = 39;
    pids[n].pid = 288;
    pids[n++].max = 39;
    pids[n].pid = 289;
    pids[n++].max = 39;
    pids[n].pid = 290;
    pids[n++].max = 39;
    pids[n].pid = 291;
    pids[n++].max = 39;
    pids[n].pid = 776;
    pids[n++].max = 39;
    pids[n].pid = 777;
    pids[n++].max = 39;
    pids[n].pid = 778;
    pids[n++].max = 39;
    pids[n].pid = 779;
    pids[n++].max = 39;
    pids[n].pid = 780;
    pids[n++].max = 39;
    pids[n].pid = 781;
    pids[n++].max = 39;
    pids[n].pid = 782;
    pids[n++].max = 39;
    pids[n].pid = 783;
    pids[n++].max = 39;

    for(int n=0;n<PID_COUNT;n++) {
        pids[n].enabled = 1;
        pids[n].len = pids[n].max/8 + 1;
    }
}

uint8_t
config_query_callback(uint16_t key, uint8_t *data, uint8_t *length) {
    int idx;

    /* These are the update rates */
    if(key >= 13 && key <= 24) {
         *(uint16_t *)data = update_rates[key-13];
         *length = 2;
         return 0;
    /* Input parameter / bit settings*/
    } else if(key >= 101 && key <= 124) {
        if(key % 2 == 1) { /* This is a PID */
            idx = (key-101)/2;
            data[0] = pids[inputs[idx].pid_index].pid;
            data[1] = pids[inputs[idx].pid_index].pid>>8;
        } else { /* Bit offset */
            idx = (key-100)/2 - 1;
            data[0] = inputs[idx].bit;
            data[1] = inputs[idx].bit>>8;
        }
        *length = 2;
        return 0;
    } else if(key == 255) {
        data[0] = 0;
        *length = 1;
        return 0;
    }
    return 1; /* Unknown Key */
}


uint8_t
config_set_callback(uint16_t key, uint8_t *data, uint8_t length) {
    int index;
    uint16_t idx;
    uint16_t val;

    /* These are the update rates */
    if(key >= 13 && key <= 24) {
        update_rates[key-13] = *(uint16_t *)data;
        return 0;
    /* Input parameter / bit settings*/
    } else if(key >= 101 && key <= 124) {
        val = *(uint16_t *)data;
        idx = key - 100;
        if(idx % 2 == 1) { /* This is a PID */
            mvprintw(19,2,"Checking pid = %d for input #%d\n", val, (idx-1)/2);
            refresh();
            for(int n=0;n<PID_COUNT;n++) {
                if(val == pids[n].pid) { /* Found the PID*/
                    inputs[(idx-1)/2].pid_index = n;
                    if(inputs[(idx-1)/2].bit > pids[n].max) {
                        inputs[(idx-1)/2].bit = pids[n].max; /* Make sure the bits are still in range */
                    }
                    mvprintw(20,2,"Found this one %d\n", val);
                    refresh();
                    return 0;
                }
            }
            return 3;
        } else { /* Odd ones are the bit definitions */
            if(val > pids[inputs[idx/2 - 1].pid_index].max) {
                return 3;
            }
            inputs[idx/2-1].bit = val;
            mvprintw(21,2,"Set input #%d bit = %d\n",idx/2-1, val);
            refresh();
            return 0;
        }
    } else if(key == 255) {
        if(data[0]) {
            printf("SAVING");
        }
        return 0;
    }
    return 1; /* Unknown Key*/
}

void
config_debug(void) {
    for(int n=0;n<PID_COUNT;n++) {
        printf("pid[%d] pid = %d, max = %d, len = %d\n", n, pids[n].pid, pids[n].max, pids[n].len);
    }
    for(int n=0;n<12;n++) {
        printf("input[%d] pid = %d, bit = %d\n",n, inputs[n].pid_index, inputs[n].bit);
    }
}