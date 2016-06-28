// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         June 06, 2016
* \author       _abrobotics_
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

#define VERSION                 "SH-PRO v5.3.5"

#define NUM_OF_MOTORS           2       /*!< Number of motors.*/
#define NUM_OF_SENSORS          3       /*!< Number of encoders.*/
#define NUM_OF_EMGS             2       /*!< Number of emg channels.*/
#define NUM_OF_ANALOG_INPUTS    4       /*!< Total number of analogic inputs.*/

//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 1000 Hz
#define CALIBRATION_DIV         10     /*!< Frequency divisor for hand calibration (100Hz).*/

#define DIV_INIT_VALUE          1      /*!<*/

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE                   0
#define TRUE                    1

#define DEFAULT_EEPROM_DISPLACEMENT 8   /*!< Number of pages occupied by the EEPROM.*/

#define PWM_MAX_VALUE           100     /*!< Maximum value of the PWM signal.*/

#define ANTI_WINDUP             1000    /*!< Anti windup saturation.*/ 
#define DEFAULT_CURRENT_LIMIT   1000    /*!< Default Current limit, 0 stands for unlimited.*/
    
#define CURRENT_HYSTERESIS      10      /*!< milliAmperes of hysteresis for current control.*/

#define EMG_SAMPLE_TO_DISCARD   500     /*!< Number of sample to discard before calibration.*/
#define SAMPLES_FOR_MEAN        100     /*!< Number of samples used to mean current values.*/

#define SAMPLES_FOR_EMG_MEAN    1000    /*!< Number of samples used to mean emg values.*/

#define CALIB_DECIMATION        1   
#define NUM_OF_CLOSURES         5

#define POS_INTEGRAL_SAT_LIMIT  50000000    /*!< Anti windup on position control.*/
#define CURR_INTEGRAL_SAT_LIMIT 100000      /*!< Anti windup on current control.*/

#define MIN_CURR_SAT_LIMIT      30

#define LOOKUP_DIM              6           /*!< Dimension of the current lookup table.*/

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references
/** \brief Motor Reference structure
 * 
**/
struct st_ref {
    int32 pos[NUM_OF_MOTORS];       /*!< Motor position reference.*/
    int32 curr[NUM_OF_MOTORS];      /*!< Motor current reference.*/
    int32 pwm[NUM_OF_MOTORS];       /*!< Motor direct pwm control.*/
    uint8 onoff;                    /*!< Motor drivers enable.*/
};

//=============================================================     measurements
/** \brief Measurements structure
 *
**/
struct st_meas {
    int32 pos[NUM_OF_SENSORS];      /*!< Encoder sensor position.*/
    int32 curr[NUM_OF_MOTORS];      /*!< Motor current and current estimation.*/
    int16 rot[NUM_OF_SENSORS];      /*!< Encoder sensor rotations.*/

    int32 emg[NUM_OF_EMGS];         /*!< EMG sensors values.*/
    int32 vel[NUM_OF_SENSORS];      /*!< Encoder rotational velocity.*/
    int32 acc[NUM_OF_SENSORS];      /*!< Encoder rotational acceleration.*/
};

//==============================================================     data packet
/** \brief Data sent/received structure
 *
**/
struct st_data {
    uint8   buffer[128];            /*!< Data buffer [CMD | DATA | CHECKSUM].*/
    int16   length;                 /*!< Data buffer length.*/
    int16   ind;                    /*!< Data buffer index.*/
    uint8   ready;                  /*!< Data buffer flag to see if the data is ready.*/
};

//============================================     settings stored on the memory
/** \brief Memory structure
 *
**/ 
struct st_mem {
    uint8   flag;                       /*!< If checked the device has been configured.*/                   //1
    uint8   id;                         /*!< Device id.*/                                                   //1

    int32   k_p;                        /*!< Position controller proportional constant.*/                   //4
    int32   k_i;                        /*!< Position controller integrative constant.*/                    //4
    int32   k_d;                        /*!< Position controller derivative constant.*/                     //4

    int32   k_p_c;                      /*!< Current controller proportional constant.*/                    //4
    int32   k_i_c;                      /*!< Current controller integrative constant.*/                     //4
    int32   k_d_c;                      /*!< Current controller derivative constant.*/                      //4 26

    int32   k_p_dl;                     /*!< Double loop position controller prop. constant.*/              //4
    int32   k_i_dl;                     /*!< Double loop position controller integr. constant.*/            //4
    int32   k_d_dl;                     /*!< Double loop position controller deriv. constant.*/             //4
    int32   k_p_c_dl;                   /*!< Double loop current controller prop. constant.*/               //4
    int32   k_i_c_dl;                   /*!< Double loop current controller integr. constant.*/             //4
    int32   k_d_c_dl;                   /*!< Double loop current controller deriv. constant.*/              //4 24
    
    uint8   activ;                      /*!< Startup activation.*/                                          //1
    uint8   input_mode;                 /*!< Motor Input mode.*/                                            //1
    uint8   control_mode;               /*!< Motor Control mode.*/                                          //1

    uint8   res[NUM_OF_SENSORS];        /*!< Angle resolution.*/                                            //3
    int32   m_off[NUM_OF_SENSORS];      /*!< Measurement offset.*/                                          //12
    float   m_mult[NUM_OF_SENSORS];     /*!< Measurement multiplier.*/                                      //12 30

    uint8   pos_lim_flag;               /*!< Position limit active/inactive.*/                              //1
    int32   pos_lim_inf[NUM_OF_MOTORS]; /*!< Inferior position limit for motors.*/                          //8
    int32   pos_lim_sup[NUM_OF_MOTORS]; /*!< Superior position limit for motors.*/                          //8

    int32   max_step_pos;               /*!< Maximum number of steps per cycle for positive positions.*/    //4
    int32   max_step_neg;               /*!< Maximum number of steps per cycle for negative positions.*/    //4 25

    int16   current_limit;              /*!< Limit for absorbed current.*/                                  //2

    uint16  emg_threshold[NUM_OF_EMGS]; /*!< Minimum value for activation of EMG control.*/                 //4

    uint8   emg_calibration_flag;       /*!< Enable emg calibration on startup.*/                           //1
    uint32  emg_max_value[NUM_OF_EMGS]; /*!< Maximum value for EMG.*/                                       //8

    uint8   emg_speed;                  /*!< Maximum closure speed when using EMG.*/                        //1

    uint8   double_encoder_on_off;      /*!< Double encoder ON/OFF.*/                                       //1

    int8    motor_handle_ratio;         /*!< Discrete multiplier for handle device.*/                       //1 

    uint8   activate_pwm_rescaling;     /*!< Activation of PWM rescaling for 12V motors.*/                  //1 19

    float   curr_lookup[LOOKUP_DIM];    /*!< Table of values to get estimated curr.*/                       //24

                                                                                            //TOT           148 bytes
};

//=================================================     device related variables
/** \brief Device related structure
 *
**/ 
struct st_dev{
    int32   tension;                /*!< Power supply tension.*/
    float   tension_conv_factor;    /*!< Used to calculate input tension.*/
    int8    pwm_limit;              /*!< Limit on pwm value driven to the motor.*/
    uint8   tension_valid;          /*!< Flag checked if the power supply has a valid value.*/
};


//==============================================     hand calibration parameters
/** \brief Hand calibration structure
 *
**/ 
struct st_calib {
    uint8   enabled;                /*!< Calibration enabling flag.*/
    uint8   direction;              /*!< Direction of motor winding.*/
    int16   speed;                  /*!< Speed of hand opening/closing.*/
    int16   repetitions;            /*!< Number of cycles of hand closing/opening.*/
};


//====================================      external global variables definition

extern struct st_ref    g_ref;          /*!< Reference variables.*/
extern struct st_meas   g_meas;         /*!< Measurements.*/
extern struct st_data   g_rx;           /*!< Incoming/Outcoming data.*/
extern struct st_mem    g_mem, c_mem;   /*!< Memory parameters.*/
extern struct st_dev    device;         /*!< Device related variables.*/
extern struct st_calib  calib;

extern float tau_feedback;              /*!< Torque feedback.*/

extern uint32 timer_value;              /*!< Time in which the main loop is executed.*/

extern uint8 reset_last_value_flag;     /*!< This flag is set when the encoders last values must be resetted.*/
extern int8 pwm_sign;                   /*!< Sign of pwm driven. Used to obtain current sign.*/

// -----------------------------------------------------------------------------


#endif

//[] END OF FILE