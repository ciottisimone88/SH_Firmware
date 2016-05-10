// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         globals.c
*
* \brief        Global variables.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref;                  // motor variables
struct st_meas  g_meas;                 // measurements
struct st_data  g_rx;                   // income data
struct st_mem   g_mem, c_mem;           // memory
struct st_dev   device;                 // device related variables
struct st_calib calib;

float tau_feedback;

// utility timer value for display

uint32 timer_value;

uint8 reset_last_value_flag;
int8 pwm_sign;

/* END OF FILE */