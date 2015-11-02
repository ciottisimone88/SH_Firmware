// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         utils.h
*
* \brief        Declaration of utility functions.
* \date         Feb 16, 2014
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/


#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <globals.h>

//------------------------------------------------------------------     DEFINES

#define TIMER_CLOCK 10000
#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))

#define ALPHA 50   	// emg filter
#define BETA  8    	// current filter
#define GAMMA 32   	// velocity filter
#define DELTA 32	// acceleration filter

//-------------------------------------------------------------     DECLARATIONS

// Tension filter
int32 filter_v(int32 new_value);
// Emg fitlers
int32 filter_ch1(int32 value);
int32 filter_ch2(int32 value);
// Motor current filter
int32 filter_i1(int32 value);
// Velocity Filters
int32 filter_vel_1(int32 value);
int32 filter_vel_2(int32 value);
int32 filter_vel_3(int32 value);
// Acceleration filters
int32 filter_acc_1(int32 value);
int32 filter_acc_2(int32 value);
int32 filter_acc_3(int32 value);
// Function used to obtain current estimation through current lookup table
int32 curr_estim(int32 pos, int32 vel, int32 accel);
// Low pass filter on current difference between measured and estimated current 
int32 filter_curr_diff(int32);

int round(double x);
uint32 my_mod(int32 val, int32 divisor);
uint8 check_enc_data(uint32*);

int calc_turns_fcn(int32 pos1, int32 pos2);

void calibration(void);
void torque_feedback(void);

#endif

/* [] END OF FILE */
