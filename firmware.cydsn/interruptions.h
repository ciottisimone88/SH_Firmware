// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED

//==================================================================     include
#include <device.h>

//====================================================     interrupt declaration
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);

//=====================================================     function declaration
void function_scheduler(void);

void analog_read_init(uint8 index);
void analog_read_end(uint8 index);
void encoder_reading(uint8 index);
void motor_control(void);
void pwm_limit_search(void);

void overcurrent_control(void);

// ----------------------------------------------------------------------------

#endif

/* [] END OF FILE */