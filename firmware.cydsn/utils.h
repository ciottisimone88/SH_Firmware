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


//-------------------------------------------------------------     DECLARATIONS

int32 filter_v(int32 new_value);
int32 filter_ch1(int32 value);
int32 filter_ch2(int32 value);
int32 filter_i1(int32 value);

int round(double x);
uint32 my_mod(int32 val, int32 divisor);
uint8 check_enc_data(uint32*);

int calc_turns_fcn(int32 pos1, int32 pos2);

void calibration(void);

void torque_feedback(void);

#endif

/* [] END OF FILE */
