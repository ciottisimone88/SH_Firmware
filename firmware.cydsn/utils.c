// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         utils.h
*
* \brief        Definition of utility functions.
* \date         Feb 16, 2014
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#include <utils.h>
#include <math.h>

//--------------------------------------------------------------     DEFINITIONS

#define ALPHA 50 // emg filter
#define BETA  8 // current filter

#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))


int32 filter_v(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}


int32 filter_ch1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

    old_value = aux;

    return aux;
}

int32 filter_ch2(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

    old_value = aux;

    return aux;
}

int32 filter_i1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}


//==============================================================================
//                                                                  BIT CHECKSUM
//==============================================================================


uint8 BITChecksum(uint32 mydata) {
    uint8 i;
    uint8 checksum = 0;
    for(i = 0; i < 31; ++i) {
        checksum = checksum ^ (mydata & 1);
        mydata = mydata >> 1;
    }
    return checksum;
}

//==============================================================================
//                                                                ROUND_FUNCTION
//==============================================================================

int round(double x) {

    if (x < 0.0)
        return (int)(x - 0.5);
    else
        return (int)(x + 0.5);
}

//==============================================================================
//                                                                        MODULE
//==============================================================================

int32 my_mod(int32 val, int32 divisor) {

    if (val >= 0) {
        return (int32)(val % divisor);
    } else {
        return (int32)(divisor - (-val % divisor));
    }
}


//==============================================================================
//                                                                     CALIBRATE
//==============================================================================

void calibration(void) {
    static uint8 direction; //0 closing, 1 opening
    static uint8 closure_counter;


    // closing
    if (direction == 0) {
        g_ref.pos[0] += dx_sx_hand * (calib.speed << g_mem.res[0]);
        if ((g_ref.pos[0] * dx_sx_hand) > closed_hand_pos) {
            direction = 1;
        }
    } else { //opening
        g_ref.pos[0] -= dx_sx_hand * (calib.speed << g_mem.res[0]);
        if (SIGN(g_ref.pos[0]) != dx_sx_hand) {
            direction = 0;
            closure_counter++;
            if (closure_counter == calib.repetitions) {
                closure_counter = 0;
                calib.enabled = FALSE;
            }
        }
    }
}

/* [] END OF FILE */