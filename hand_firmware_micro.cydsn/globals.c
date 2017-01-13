// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         globals.c
*
* \brief        Global variables.
* \date         June 06, 2016
* \author       _qbrobotics_
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref, g_refNew, g_refOld;  // motor variables
struct st_meas  g_meas, g_measOld;          // measurements
struct st_data  g_rx;                       // income data
struct st_mem   g_mem, c_mem;               // memory
struct st_calib calib;

float tau_feedback;

// Timer value for debug field

uint32 timer_value;
uint32 timer_value0;

// Device Data

int32   dev_tension;                // Power supply tension
uint8   dev_pwm_limit;

// Bit Flag

CYBIT reset_last_value_flag;
CYBIT tension_valid;
CYBIT interrupt_flag = FALSE;
CYBIT watchdog_flag = FALSE;

// DMA Buffer

int16 ADC_buf[4]; 

// PWM value
int8 pwm_sign;


// Hand comparaison variables
int16 pwm_value;
int32 hand_comp_closure;
uint32 hand_comp_reactivation_time;
uint8 hand_comp_current_mode;
struct st_calib hand_comp_calib;
float hand_comp_closure_threshold;

uint32 timer_val;
uint32 timer_val_init;

/* END OF FILE */