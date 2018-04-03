// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2017, qbrobotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------
/**
* \file         utils.c
*
* \brief        Definition of utility functions.
* \date         June 06, 2016
* \author       _qbrobotics_
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#include <utils.h>
#include <math.h>

//==============================================================================
//                                                            Current Estimation
//==============================================================================

int32 curr_estim ( int32 CYDATA pos, int32 CYDATA vel, int32 CYDATA ref) {

    static int32 CYDATA virtual_pos_friction; 	//Virtual position used to estimate friction
    static int32 CYDATA err_pos_dt;			    //Temporal evolution of error position.
										//Needed to model the current transients due to reference steps.

    int32 CYDATA curr_estimate;

    if (pos < ZERO_TOL) 
		virtual_pos_friction = pos;
    else {
		if ((pos - virtual_pos_friction) > ZMAX) 
			virtual_pos_friction =  pos - ZMAX;
    	else { 
			if ((pos - virtual_pos_friction) < -ZMAX) 
				virtual_pos_friction =  pos + ZMAX;
		}
	}        
   
    curr_estimate = pos * g_mem.curr_lookup[0] * (1 + pos * g_mem.curr_lookup[1]) + (pos - virtual_pos_friction) * (g_mem.curr_lookup[2] / ZMAX) + vel * g_mem.curr_lookup[3] * (1 + abs(vel) * g_mem.curr_lookup[4]) + (ref - err_pos_dt) * g_mem.curr_lookup[5];
    
    if (curr_estimate > c_mem.current_limit) 
        curr_estimate = c_mem.current_limit;
    else {
		if (curr_estimate < -c_mem.current_limit)
        	curr_estimate = -c_mem.current_limit;
	}

    err_pos_dt = (REFSPEED * ref + (1024 - REFSPEED) * err_pos_dt) / 1024;         
    
    return curr_estimate;        
}

//==============================================================================
//                                                   Voltage and current filters
//==============================================================================

int32 filter_v(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + (new_value << 6) * (ALPHA)) /1024;

    old_value = aux;

    return (aux /64);
}

int32 filter_i1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + (new_value << 6) * (ALPHA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                              First Emg Filter
//==============================================================================

int32 filter_ch1(int32 new_value) {

   static int32 old_value, aux;    
     
    if (new_value < 0)
        new_value = 0;

    aux = (old_value * (1024 - BETA) + (new_value << 6) * (BETA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                              Velocity filters
//==============================================================================

int32 filter_vel_1(int32 new_value) {

   static int32 old_value, aux;

    aux = (old_value * (1024 - GAMMA) + (new_value << 6) * (GAMMA)) /1024;

    old_value = aux;

    return (aux /64);
}

int32 filter_vel_2(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - GAMMA) + (new_value << 6) * (GAMMA)) /1024;

    old_value = aux;

    return (aux /64);
}

int32 filter_vel_3(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - GAMMA) + (new_value << 6) * (GAMMA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                     Current difference filter
//==============================================================================

int32 filter_curr_diff(int32 new_value) {

    static int32 old_value, aux;
    
    aux = (old_value * (1024 - ETA) + (new_value << 6) * (ETA)) /1024;

    old_value = aux;

    return (aux /64);

}

//==============================================================================
//                                                             Second Emg Filter
//==============================================================================

int32 filter_ch2(int32 new_value) {

    static int32 old_value, aux;
    
    if (new_value < 0)
        new_value = 0;

    aux = (old_value * (1024 - BETA) + (new_value << 6) * (BETA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                          Acceleration filters
//==============================================================================

int32 filter_acc_1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - DELTA) + (new_value << 6) * (DELTA)) /1024;

    old_value = aux;

    return (aux /64);
}

int32 filter_acc_2(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - DELTA) + (new_value << 6) * (DELTA)) /1024;

    old_value = aux;

    return (aux /64);
}

int32 filter_acc_3(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - DELTA) + (new_value << 6) * (DELTA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                                CHECK ENC DATA
//==============================================================================

// Returns 1 if the encoder data is correct, 0 otherwise

CYBIT check_enc_data(const uint32 *value) {

    const uint8* CYIDATA p = (const uint8*)value;
    uint8 CYDATA a = *p;

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

int my_round(const double x) {

    if (x < 0.0)
        return (int)(x - 0.5);
    else
        return (int)(x + 0.5);
}

//==============================================================================
//                                                                        MODULE
//==============================================================================

uint32 my_mod(int32 val, int32 divisor) {

    if (val >= 0) {
        return (int32)(val % divisor);
    } else {
        return (int32)((divisor - (-val % divisor)) % divisor);
    }
}


//==============================================================================
//                                                                     CALIBRATE
//==============================================================================

void calibration(void) {
    static uint8 direction;                 //0 closing, 1 opening
    static uint16 closure_counter;          //Range [0 - 2^16]
    static CYBIT count_one = 1;                 //Used to count only one cycle, when inputs are generated
                                            //and pass the threshold, having the condition verified


    // closing
    if (direction == 0) {
        g_refNew.pos[0] += (calib.speed << g_mem.res[0]);
        if((g_refNew.pos[0] >> g_mem.res[0]) >= cycles_thr && count_one && g_ref.onoff) {
            cycles_reader += 1;
            count_one = 0;          //Necessary because it counted cycles twice
        }
        if ((g_refNew.pos[0]) > g_mem.pos_lim_sup[0]) {
            direction = 1;
        }
    } else { //opening
        g_refNew.pos[0] -= (calib.speed << g_mem.res[0]);
        if (SIGN(g_refNew.pos[0]) != 1) {
            direction = 0;
            closure_counter++;
            count_one = 1;          // Enables back the cycles counter
            if (closure_counter == calib.repetitions) {
                closure_counter = 0;
                calib.enabled = FALSE;
            }
        }
    }
}


//==============================================================================
//                                                      DOUBLE ENCODER CALC TURN
//==============================================================================

// Use this matlab function to calculate I1 and I2
//
// function [inv_a, inv_b] = mod_mul_inv(a, b)
//     a = int32(a);
//     b = int32(b);
//     if b == 0
//         inv_a = 1;
//         inv_b = 0;
//         return
//     else
//         q = idivide(a,b,'floor');
//         r = mod(a,b);
//         [s, t] = mod_mul_inv(b, r);
//     end
//     inv_a = t;
//     inv_b = s - q * t;
// return

// Number of teeth of the two wheels
#define N1 15           ///< Teeth of the first encoder wheel
#define N2 14           ///< Teeth of the second encoder wheel

#define I1 1            ///< First wheel invariant value
#define I2 (-1)         ///< Second wheel invariant value

// Number of ticks per turn
#define M 65536          ///< Number of encoder ticks per turn


int calc_turns_fcn(const int32 pos1, const int32 pos2) {
    
    int32 x = (my_mod( - N2*pos2 - N1*pos1, M*N2) + M/2) / M;

    int32 aux = my_mod(x*I1, N2);
    
    return (my_mod(aux + N2/2, N2) - N2/2);
}

/* [] END OF FILE */