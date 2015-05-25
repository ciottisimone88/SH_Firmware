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
#define BETA  32 // current filter

#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))


int32 filter_v(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
}

int32 filter_i1(int32 new_value) {

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

//==============================================================================
//                                                                CHECK ENC DATA
//==============================================================================

// Returns 1 if the encoder data is correct, 0 otherwise

uint8 check_enc_data(uint32 *value) {

    const uint8* p = (const uint8*)value;
    uint8 a = *p;

    a = a ^ *(++p);
    a = a ^ *(++p);
    a = a ^ *(++p);
    a = (a & 0x0F) ^ (a>>4);

    return (0x9669 >> a) & 0x01;
    //0x9669 is a bit vector representing the !(bitwise XOR) of 4bits
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