// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         Jul 29, 2013
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "commands.h"

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION                 "SH-PRO v5.3.0"

#define NUM_OF_MOTORS           2
#define NUM_OF_SENSORS          3
#define NUM_OF_EMGS             2
#define NUM_OF_ANALOG_INPUTS    4

//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 1000 Hz
#define CALIBRATION_DIV         10     // 100 Hz

#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE                   0
#define TRUE                    1

#define DEFAULT_EEPROM_DISPLACEMENT 8   // in pages

#define PWM_MAX_VALUE           100     // PWM is from 0 to 100

#define ANTI_WINDUP             1000
#define DEFAULT_CURRENT_LIMIT   1000    // Current limit for hand closing
                                        // 0 means unlimited
#define CURRENT_HYSTERESIS      10      // mA of hysteresis for current control

#define EMG_SAMPLE_TO_DISCARD   500     // Number of sample to discard before calibration
#define SAMPLES_FOR_MEAN        100

#define SAMPLES_FOR_EMG_MEAN    1000

#define CALIB_DECIMATION        1
#define NUM_OF_CLOSURES         5

#define POS_INTEGRAL_SAT_LIMIT  50000000
#define CURR_INTEGRAL_SAT_LIMIT 10000

#define MIN_CURR_SAT_LIMIT      30

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

struct st_ref {
    int32 pos[NUM_OF_MOTORS];       // motor reference position
    uint8 onoff;                    // enable flags
};

//=============================================================     measurements

struct st_meas {
    int32 pos[NUM_OF_SENSORS];      // sensor position

    int32 curr[NUM_OF_MOTORS];      // motor currents

    int16 rot[NUM_OF_SENSORS];      // sensor rotations

    int32 emg[NUM_OF_EMGS];         // EMG values
};

//==============================================================     data packet

struct st_data {
    uint8   buffer[128];            // CMD/DATA/CHECKSUM
    int16   length;                 // length
    int16   ind;                    // index
    uint8   ready;                  // Flag
};

//============================================     settings stored on the memory

struct st_mem {
    uint8   flag;                       // Device has been configured               1
    uint8   id;                         // device id                                1

    int32   k_p;                        // Proportional constant                    4
    int32   k_i;                        // Derivative constant                      4
    int32   k_d;                        // Integrative constant                     4

    int32   k_p_c;                      // Proportional constant curr               4
    int32   k_i_c;                      // Derivative constant curr                 4
    int32   k_d_c;                      // Integrative constant curr                4 26

    uint8   activ;                      // Activation upon startup                  1
    uint8   input_mode;                 // Input mode                               1
    uint8   control_mode;               // Control mode                             1

    uint8   res[NUM_OF_SENSORS];        // Angle resolution                         3
    int32   m_off[NUM_OF_SENSORS];      // Measurement offset                       12
    float   m_mult[NUM_OF_SENSORS];     // Measurement multiplier                   12 30

                                        // Absolute position limits
    uint8   pos_lim_flag;               // Position limit active/inactive           1
    int32   pos_lim_inf[NUM_OF_MOTORS]; // Inferior position limit for motors       8
    int32   pos_lim_sup[NUM_OF_MOTORS]; // Superior position limit for motors       8

    int32   max_step_pos;               // Maximum number of step per cylce when    4
    int32   max_step_neg;               // using sensor 2 as input                  4 25

    int16   current_limit;              // Limit for absorbed current               2

    uint16  emg_threshold[NUM_OF_EMGS]; // Minimum value for activation             4

    uint8   emg_calibration_flag;       // Enable emg calibration on startup        1
    uint32  emg_max_value[NUM_OF_EMGS]; // Maximum value for EMG                    8

    uint8   emg_speed;                  // Maximum closure speed when using emg     1

    uint8   double_encoder_on_off;      // Double encoder ON/OFF                    1

    int8    motor_handle_ratio;         // Discrete multiplier for handle           1 

    uint8   activate_pwm_rescaling;     // PWM rescaling for 12V motors             1 19

                                                                    //TOT           100 bytes
};

//=================================================     device related variables

struct st_dev{
    int32   tension;                // Power supply tension
    float   tension_conv_factor;    // Used to calculate input tension
    int8    pwm_limit;
    uint8   tension_valid;
};


//==============================================     hand calibration parameters

struct st_calib {
    uint8   enabled;
    uint8   direction;
    int16   speed;
    int16   repetitions;
};


//====================================      external global variables definition

extern struct st_ref    g_ref;          // motor variables
extern struct st_meas   g_meas;         // measurements
extern struct st_data   g_rx;           // income data
extern struct st_mem    g_mem, c_mem;   // memory
extern struct st_dev    device;         // device related variables
extern struct st_calib  calib;

extern int32 opened_hand_pos;
extern int32 closed_hand_pos;
extern int8 dx_sx_hand;

extern float tau_feedback;

extern uint32 timer_value;

extern uint8 reset_last_value_flag;

// -----------------------------------------------------------------------------


#endif

//[] END OF FILE