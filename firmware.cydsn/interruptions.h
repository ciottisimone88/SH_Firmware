// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         June 06, 2016
* \author       _qbrobotics_
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED

//==================================================================     include
#include <device.h>

/** \name Interruption */
/** \{ */
//====================================================     interrupt declaration
/** This interruption unpacks the packet received and, accordingly to the command
 * 	received, selects the correct function to be called
**/
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);
/** \} */

//=====================================================     function declaration

/** \name General function scheduler */
/** \{ */
//=====================================================     function_scheduler
/** This function schedules the other functions in an order that optimizes the 
 *	controller usage.	
**/
void function_scheduler();
/** \} */

/** \name Analog variables reading functions */
/** \{ */
//=====================================================     analog_read_init
/** This function starts converting the analog data pointed by index.
 *
 *	\param index 	The index of the data that needs to be converted.
**/
void analog_read_init(uint8 index);

//=====================================================     analog_read_end
/** This functions ends the conversion started by \ref analog_read_init 
 *	"analog_read_init".
 *
 *	\param index 	The index of the data converted.
**/
void analog_read_end(uint8 index);
/** \} */


/** \name Encoder reading function */
/** \{ */
//=====================================================     encoder_reading
/** This functions reads the value from the encoder pointed by index.
 *
 *	\param index 	The number of the encoder that must be read.
**/
void encoder_reading(uint8 index);
/** \} */


/** \name Motor control function */
/** \{ */
//=====================================================     motor_control
/** This function controls the motor direction and velocity, depending on 
 * 	the input and control modality set. 
**/
void motor_control();
/** \} */


/** \name Utility functions */
/** \{ */
//=====================================================     pwm_limit_search
/** This function scales the pwm value of the motor, depending on the power 
 * 	supply voltage, in order to not make the motor wind too fast.
**/
void pwm_limit_search();

//=====================================================     overcurrent_control
/** This function increases or decreases the pwm value, depending on the current
 * 	absorbed by the motor.
**/
void overcurrent_control();

/** \} */

#endif

/* [] END OF FILE */