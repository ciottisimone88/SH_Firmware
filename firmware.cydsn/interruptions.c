// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.c
*
* \brief        Interruption functions are in this file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>
#include <globals.h>
#include <utils.h>


//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
//
// - 0:     Waits for beggining characters
// - 1:     Waits for ID;
// - 2:     Data length;
// - 3:     Receive all bytes;
// - 4:     Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt){

//===============================================     local variables definition

    static uint8    state = 0;                          // actual state
    static struct   st_data data_packet;                // local data packet
    static uint8    rx_queue[3];                        // last 3 bytes received
    static uint8    rx_data;                            // RS485 UART rx data
    static uint8    rx_data_type;                       // packet for me or not


//==========================================================     receive routine

// get data while rx fifo is not empty
    while (UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_FIFO_NOTEMPTY) {
        rx_data = UART_RS485_GetChar();

        switch (state) {
            // ----- wait for frame start -----
            case 0:

                rx_queue[0] = rx_queue[1];
                rx_queue[1] = rx_queue[2];
                rx_queue[2] = rx_data;

                // Finding starting frame
                if ((rx_queue[1] == ':') && (rx_queue[2] == ':')) {
                    rx_queue[0] = 0;
                    rx_queue[1] = 0;
                    rx_queue[2] = 0;
                    state       = 1;
                } else if(  //this has to be removed
                        (rx_queue[0] == 63) &&      //ASCII - ?
                        (rx_queue[1] == 13) &&      //ASCII - CR
                        (rx_queue[2] == 10)){       //ASCII - LF
                    infoSend();
                }
                break;

            // ----- wait for id -----
            case 1:

                // packet is for my ID or is broadcast
                if((rx_data == c_mem.id) || (rx_data == 0)) {
                    rx_data_type = 0;
                } else {                //packet is for others
                    rx_data_type = 1;
                }
                data_packet.length = -1;
                state = 2;
                break;

            // ----- wait for length -----
            case 2:

                data_packet.length = rx_data;
                // check validity of pack length
                if (data_packet.length <= 1) {
                    data_packet.length = -1;
                    state = 0;
                } else if (data_packet.length > 128) {
                    data_packet.length = -1;
                    state = 0;
                } else {
                    data_packet.ind = 0;
                    if(rx_data_type == 0) {
                        state = 3;          // packet for me or boradcast
                    } else {
                        state = 4;          // packet for others
                    }
                }
                break;

            // ----- receving -----
            case 3:

                data_packet.buffer[data_packet.ind] = rx_data;
                data_packet.ind++;

                // check end of transmission
                if (data_packet.ind >= data_packet.length) {
                    // verify if frame ID corresponded to the device ID
                    if (rx_data_type == 0) {
                        // copying data from buffer to global packet
                        memcpy(g_rx.buffer, data_packet.buffer, data_packet.length);
                        g_rx.length = data_packet.length;
                        commProcess();
                    }
                    data_packet.ind    =  0;
                    data_packet.length = -1;
                    state              =  0;
                }
                break;

            // ----- other device is receving -----
            case 4:

                if(!(--data_packet.length)) {
                    data_packet.ind    = 0;
                    data_packet.length = -1;
                    RS485_CTS_Write(1);
                    RS485_CTS_Write(0);
                    state              = 0;
                }
                break;
        }
    }

    /* PSoC3 ES1, ES2 RTC ISR PATCH  */
    #if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
        #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_RS485_RX__ES2_PATCH ))
            ISR_MOTORS_CONTROL_ISR_PATCH();
        #endif
    #endif
}

//==============================================================================
//                                                            FUNCTION SCHEDULER
//==============================================================================
// Call all the function with the right frequency
//==============================================================================

void function_scheduler(void) {
    // Base frequency 1000 Hz

    static uint8 counter_analog_measurements    = DIV_INIT_VALUE;
    static uint8 counter_encoder_read           = DIV_INIT_VALUE;
    static uint8 counter_motor_control          = DIV_INIT_VALUE;
    static uint8 counter_overcurrent            = DIV_INIT_VALUE;
    static uint16 counter_calibration           = DIV_INIT_VALUE;

    static uint16 timer_counter = 1;


    if (counter_analog_measurements == ANALOG_MEASUREMENTS_DIV) {
        analog_measurements();
        counter_analog_measurements = 0;
    }
    counter_analog_measurements++;


    if (counter_encoder_read == ENCODER_READ_DIV) {
        encoder_reading();
        counter_encoder_read = 0;
    }
    counter_encoder_read++;


    if (counter_motor_control == MOTOR_CONTROL_DIV) {
        motor_control();
        counter_motor_control = 0;
    }
    counter_motor_control++;


    if (counter_overcurrent == OVERCURRENT_DIV) {
        overcurrent_control();
        counter_overcurrent = 0;
    }
    counter_overcurrent++;



    // Divider 10, freq = 500 Hz
    if (calib.enabled == TRUE) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }


    // Use MY_TIMER to store the execution time of 5000 executions
    // to check the base frequency

    // if (timer_counter < 4) {
    //     timer_counter++;
    // } else if (timer_counter == 4) {
        timer_value = (uint32)MY_TIMER_ReadCounter();
        MY_TIMER_WriteCounter(5000000);
    //     timer_counter = 1;
    // }
}


//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control(void) {

    static int32 input_1 = 0;
    static int32 input_2 = 0;

    static int32 pos_prec_1, pos_prec_2;
    static int32 error_1, error_2;

    static int32 err_sum_1, err_sum_2;

    static int32 err_emg_1, err_emg_2;

    static uint8 current_emg = 0;   // 0 NONE
                                    // 1 EMG 1
                                    // 2 EMG 2
                                    // wait for both to get down


    err_emg_1 = g_meas.emg[0] - c_mem.emg_threshold[0];
    err_emg_2 = g_meas.emg[1] - c_mem.emg_threshold[1];


    switch(c_mem.input_mode) {

        case INPUT_MODE_ENCODER3:
            //--- speed control in both directions ---//

            // motor 1
            if (((g_meas.pos[2] - g_ref.pos[0]) > c_mem.max_step_pos)   &&   (c_mem.max_step_pos != 0)) {
                g_ref.pos[0] += c_mem.max_step_pos;
            } else if (((g_meas.pos[2] - g_ref.pos[0]) < c_mem.max_step_neg)   &&   (c_mem.max_step_neg != 0)) {
                g_ref.pos[0] += c_mem.max_step_neg;
            } else {
                g_ref.pos[0] = g_meas.pos[2];
            }
            break;


        case INPUT_MODE_EMG_PROPORTIONAL:
            if (err_emg_1 > 0) {
            g_ref.pos[0] = (err_emg_1 * dx_sx_hand * closed_hand_pos) / (1024 - c_mem.emg_threshold[0]);
            } else {
                g_ref.pos[0] = 0;
            }
            break;


        case INPUT_MODE_EMG_INTEGRAL:
            if (err_emg_1 > 0) {
                g_ref.pos[0] += (err_emg_1 * dx_sx_hand * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[0]);
            }
            if (err_emg_2 > 0) {
                g_ref.pos[0] -= (err_emg_2 * dx_sx_hand * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[1]);
            }
            break;

        case INPUT_MODE_EMG_FCFS:
            switch (current_emg) {
                case 0:
                    // Look for the first EMG passing the threshold
                    if (err_emg_1 > 0) {
                        current_emg = 1;
                        break;
                    }
                    if (err_emg_2 > 0) {
                        current_emg = 2;
                        break;
                    }
                    break;

                case 1:
                    // EMG 1 is first
                    if (err_emg_1 < 0) {
                        current_emg = 0;
                        break;
                    }

                    g_ref.pos[0] += (err_emg_1 * dx_sx_hand * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[0]);
                    break;

                case 2:
                    // EMG 2 is first
                    if (err_emg_2 < 0) {
                        current_emg = 0;
                        break;
                    }

                    g_ref.pos[0] -= (err_emg_2 * dx_sx_hand * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[1]);
                    break;

                default:
                    break;
            }

            break;

        case INPUT_MODE_EMG_FCFS_ADV:
            switch (current_emg) {
                // Look for the first EMG passing the threshold
                case 0:
                    if (err_emg_1 > 0) {
                        current_emg = 1;
                        break;
                    }
                    if (err_emg_2 > 0) {
                        current_emg = 2;
                        break;
                    }
                    break;

                // EMG 1 is first
                case 1:
                    // If both signals are under threshold go back to status 0
                    if ((err_emg_1 < 0) && (err_emg_2 < 0)) {
                        current_emg = 0;
                        break;
                    }

                    // but if the current signal come back over threshold, continue using it
                    if (err_emg_1 > 0) {
                        g_ref.pos[0] += (err_emg_1 * dx_sx_hand * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[0]);
                    }
                    break;

                // EMG 2 is first
                case 2:
                    // If both signals are under threshold go back to status 0
                    if ((err_emg_1 < 0) && (err_emg_2 < 0)) {
                        current_emg = 0;
                        break;
                    }

                    // but if the current signal come back over threshold, continue using it
                    if (err_emg_2 > 0) {
                        g_ref.pos[0] -= (err_emg_2 * dx_sx_hand * c_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[1]);
                    }
                    break;

                default:
                    break;
            }


        default:
            break;
    }


    // Position limit
    if (c_mem.pos_lim_flag) {
        if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];

        if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
    }

    //////////////////////////////////////////////////////////     CONTROL_ANGLE

    #if (CONTROL_MODE == CONTROL_ANGLE)
        error_1 = g_ref.pos[0] - g_meas.pos[0];

        err_sum_1 += error_1;

        // anti-windup (for integral control)
        if (err_sum_1 > ANTI_WINDUP) {
            err_sum_1 = ANTI_WINDUP;
        } else if (err_sum_1 < -ANTI_WINDUP) {
            err_sum_1 = -ANTI_WINDUP;
        }

        // Proportional
        if (c_mem.k_p != 0) {
            input_1 = (int32)(c_mem.k_p * error_1) >> 16;
        }

        // Integral
        if (c_mem.k_i != 0) {
            input_1 += (int32)(c_mem.k_i * err_sum_1) >> 16;
        }

        // Derivative
        if (c_mem.k_d != 0) {
            input_1 += (int32)(c_mem.k_d * (pos_prec_1 - g_meas.pos[0])) >> 16;
        }

        // Update measure
        pos_prec_1 = g_meas.pos[0];

    #endif

    ////////////////////////////////////////////////////////     CONTROL_CURRENT

    #if (CONTROL_MODE == CONTROL_CURRENT)
        if(g_ref.onoff & 1) {
            error_1 = g_ref.pos[0] - g_meas.curr[0];

            err_sum_1 += error_1;

            input_1 += ((c_mem.k_p * (error_1)) / 65536) + err_sum_1;
        } else {
            input_1 = 0;
        }
    #endif

    ////////////////////////////////////////////////////////////     CONTROL_PWM

    #if (CONTROL_MODE == CONTROL_PWM)
        // Shift right by resolution to have the real input number
        input_1 = g_ref.pos[0] >> g_mem.res[0];
    #endif

    ////////////////////////////////////////////////////////////////////////////

    #if PWM_DEAD != 0
        if (input_1 > 0) {
            input_1 += PWM_DEAD;
        } else if (input_1 < 0) {
            input_1 -= PWM_DEAD;
        }
    #endif

    if(input_1 >  PWM_MAX_VALUE) input_1 =  PWM_MAX_VALUE;
    if(input_1 < -PWM_MAX_VALUE) input_1 = -PWM_MAX_VALUE;

    MOTOR_DIR_Write((input_1 >= 0) + ((input_2 >= 0) << 1));
    //MOTOR_DIR_Write((input_1 <= 0) + ((input_2 <= 0) << 1));

    #if (CONTROL_MODE != CONTROL_PWM)
        input_1 = (((input_1 * 1024) / PWM_MAX_VALUE) * device.pwm_limit) / 1024;
    #endif

    PWM_MOTORS_WriteCompare1(abs(input_1));
}


//==============================================================================
//                                                               ENCODER READING
//==============================================================================

void encoder_reading(void) {

    static uint8 i;      //iterator

    static int32 data_encoder;
    static int32 value_encoder;
    static int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS - 2];

    static uint8 only_first_time = 1;
    static uint8 one_time_execute = 0;

    // int calc_turns;


    // Discard first reading
    if (only_first_time) {
        for (i = 0; i < NUM_OF_SENSORS - 2; i++) {
            switch(i) {
                case 0: {
                    data_encoder = SHIFTREG_ENC_1_ReadData();
                    break;
                }
                case 1: {
                    data_encoder = SHIFTREG_ENC_2_ReadData();
                    break;
                }
            }
        }
        only_first_time = 0;
        CyDelay(1); //Wait to be sure the shift register is updated with a new valid measure
    }

    // Normal execution
    for (i = 0; i < NUM_OF_SENSORS - 2; i++) {
        switch(i) {
            case 0: {
                data_encoder = SHIFTREG_ENC_1_ReadData();
                break;
            }
            case 1: {
                data_encoder = SHIFTREG_ENC_2_ReadData();
                break;
            }
        }


        aux = data_encoder & 262142;
        if ((data_encoder & 0x01 ) == BITChecksum(aux))
        {
            aux = data_encoder & 0x3FFC0;            // reset last 6 bit
            value_encoder = (aux - 0x20000) >> 2;    // subtract half of max value
                                                        // and shift to have 16 bit val

            value_encoder  = (int16)(value_encoder + g_mem.m_off[i]);

            // take care of rotations
            aux = value_encoder - last_value_encoder[i];
            if (aux > 32768)
                g_meas.rot[i]--;
            if (aux < -32768)
                g_meas.rot[i]++;

            last_value_encoder[i] = value_encoder;

            value_encoder += g_meas.rot[i] * 65536;

            value_encoder *= c_mem.m_mult[i];
        }

        g_meas.pos[i] = value_encoder;
    }


    // if (one_time_execute < 10) {
    //     if (one_time_execute < 9) {
    //         one_time_execute++;
    //     } else {
    //         calc_turns = my_mod(round(((my_mod((g_meas.pos[0] + g_meas.pos[1]), 65536) * 28) - g_meas.pos[0]) / 65536.0), 28);

    //         if (calc_turns == 27) {
    //             g_meas.rot[0] = -1;
    //         } else {
    //             g_meas.rot[0] = calc_turns;
    //         }

    //         one_time_execute = 10;
    //     }
    // }
}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

void analog_measurements(void) {

    static uint8 i;
    static int32 value, i_aux;

    static uint16 emg_counter_1 = 0;
    static uint16 emg_counter_2 = 0;

    static uint8 emg_1_status = 1;  // 0 normal execution
    static uint8 emg_2_status = 1;  // 1 reset status
                                    // 2 discard values
                                    // 3 sum values and mean
                                    // 4 wait

    for (i = 0; i < NUM_OF_ANALOG_INPUTS; i++) {
        AMUX_FastSelect(i);

        ADC_StartConvert();
        if (ADC_IsEndConversion(ADC_WAIT_FOR_RESULT)) {

            value = (int32) ADC_GetResult16();
            ADC_StopConvert();

            switch(i) {

                // --- Input tension ---
                case 0:
                    device.tension = filter_v((value - 1638) * device.tension_conv_factor);
                    //until there is no valid input tension repeat this measurement
                    if (device.tension < 0) {
                        i = NUM_OF_ANALOG_INPUTS;
                        emg_1_status = 1;   // reset status
                        emg_2_status = 1;

                        if (c_mem.emg_calibration_flag) {
                            if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
                                (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
                                (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
                                (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
                                g_ref.onoff = 0x00;
                                MOTOR_ON_OFF_Write(g_ref.onoff);
                            }
                        }
                    }
                    break;

                // --- Current motor 1 ---
                case 1:
                    g_meas.curr[0] =  filter_i1(abs(((value - 1638) * 4000) / (1638)));
                    break;

                // --- EMG 1 ---
                case 2:
                    // if calibration is not needed go to "normal execution"
                    if (!g_mem.emg_calibration_flag) {
                        emg_1_status = 0; //normal execution
                    }
                    // EMG 1 calibration state machine
                    switch(emg_1_status) {
                        case 0: // normal execution
                            i_aux = filter_ch1(value);
                            i_aux = (1024 * i_aux) / g_mem.emg_max_value[0];

                            if (i_aux < 0) {
                                i_aux = 0;
                            } else if (i_aux > 1024) {
                                i_aux = 1024;
                            }

                            g_meas.emg[0] = i_aux;
                            break;

                        case 1: // reset variables
                            emg_counter_1 = 0;
                            g_mem.emg_max_value[0] = 0;
                            emg_1_status = 2; // goto next status
                            break;

                        case 2: // discard first EMG_SAMPLE_TO_DISCARD samples
                            emg_counter_1++;
                            if (emg_counter_1 == EMG_SAMPLE_TO_DISCARD) {
                                emg_counter_1 = 0;          // reset counter
                                LED_REG_Write(0x01);        // turn on LED
                                emg_1_status = 3;           // sum and mean status
                            }
                            break;

                        case 3: // sum first SAMPLES_FOR_EMG_MEAN samples
                            // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                            emg_counter_1++;
                            g_mem.emg_max_value[0] += filter_ch1(value);
                            if (emg_counter_1 == SAMPLES_FOR_EMG_MEAN) {
                                g_mem.emg_max_value[0] = g_mem.emg_max_value[0] / SAMPLES_FOR_EMG_MEAN; // calc mean
                                LED_REG_Write(0x00);        // led OFF
                                emg_counter_1 = 0;          // reset counter
                                emg_1_status = 0;           // goto normal execution
                            }
                            break;

                        default:
                            break;
                    }
                    break; // main switch break

                // --- EMG 2 ---
                case 3:
                    // if calibration is not needed go to "normal execution"
                    if (!g_mem.emg_calibration_flag) {
                        emg_2_status = 0; // normal execution
                    }

                    // EMG 2 calibration state machine
                    switch(emg_2_status) {
                        case 0: // normal execution
                            i_aux = filter_ch2(value);
                            i_aux = (1024 * i_aux) / g_mem.emg_max_value[1];

                            if (i_aux < 0) {
                                i_aux = 0;
                            } else if (i_aux > 1024) {
                                i_aux = 1024;
                            }

                            g_meas.emg[1] = i_aux;
                            break;

                        case 1: // reset variables
                            emg_counter_2 = 0;
                            g_mem.emg_max_value[1] = 0;
                            emg_2_status = 4; // wait for EMG 1 calibration
                            break;

                        case 2: // discard first EMG_SAMPLE_TO_DISCARD samples
                            emg_counter_2++;
                            if (emg_counter_2 == EMG_SAMPLE_TO_DISCARD) {
                                emg_counter_2 = 0;          // reset counter
                                LED_REG_Write(0x01);        // turn on LED
                                emg_2_status = 3;           // sum and mean status
                            }
                            break;

                        case 3: // sum first SAMPLES_FOR_EMG_MEAN samples
                            // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                            emg_counter_2++;
                            g_mem.emg_max_value[1] += filter_ch2(value);
                            if (emg_counter_2 == SAMPLES_FOR_EMG_MEAN) {
                                g_mem.emg_max_value[1] = g_mem.emg_max_value[1] / SAMPLES_FOR_EMG_MEAN; // calc mean
                                LED_REG_Write(0x00);        // led OFF
                                emg_counter_2 = 0;          // reset counter

                                // if EMG control mode active, activate motors accordingly with startup value
                                if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
                                    (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
                                    (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
                                    (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
                                    #if (CONTROL_MODE == CONTROL_ANGLE)
                                        g_ref.pos[0] = g_meas.pos[0];
                                        g_ref.pos[1] = g_meas.pos[1];
                                    #endif
                                    g_ref.onoff = c_mem.activ;
                                    MOTOR_ON_OFF_Write(g_ref.onoff);
                                }

                                emg_2_status = 0;           // goto normal execution
                            }
                            break;

                        case 4: // wait for EMG calibration to be done
                            if (emg_1_status == 0) {
                                emg_2_status = 2;           // goto discart sample
                            }
                            break;

                        default:
                            break;
                    }
                    break; // main switch break

                default:
                    break; // main switch break
            }
        }
    }
}

//==============================================================================
//                                                           OVERCURRENT CONTROL
//==============================================================================

void overcurrent_control(void) {
    if (c_mem.current_limit != 0) {
        // if one of the current is over the limit
        if (g_meas.curr[0] > c_mem.current_limit) {
            //decrese pwm_limit
            device.pwm_limit--;
        // if both the current are in the safe zone
        } else if (g_meas.curr[0] < (c_mem.current_limit - CURRENT_HYSTERESIS)) {
            //increase pwm_limit
            device.pwm_limit++;
        }

        // bound pwm_limit
        if (device.pwm_limit < 0) {
            device.pwm_limit = 0;
        } else if (device.pwm_limit > 100) {
            device.pwm_limit = 100;
        }
    }
}

/* [] END OF FILE */