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

// PWM values needed to obtain 11.5 V given a certain input tension
// Numbers are sperimentally calculated //[index] (millivolts)
static const uint8 pwm_preload_values[29] = {100,    //0 (11500)
                                              83,
                                              78,
                                              76,
                                              74,
                                              72,    //5 (14000)
                                              70,
                                              68,
                                              67,
                                              65,
                                              64,    //10 (16500)
                                              63,
                                              62,
                                              61,
                                              60,
                                              59,    //15 (19000)
                                              58,
                                              57,
                                              56,
                                              56,
                                              55,    //20 (21500)
                                              54,
                                              54,
                                              53,
                                              52,
                                              52,    //25 (24000)
                                              52,
                                              51,
                                              51};   //28 (25500)

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
                        state = 3;          // packet for me or broadcast
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
                    RS485_CTS_Write(1);             //CTS on falling edge
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

    static uint16 counter_calibration = DIV_INIT_VALUE;

    MY_TIMER_WriteCounter(5000000);

    analog_read_init(0);
    encoder_reading(0);
    analog_read_end(0);

    analog_read_init(1);
    encoder_reading(1);
    analog_read_end(1);

    analog_read_init(2);
    encoder_reading(2);
    analog_read_end(2);

    analog_read_init(3);
    motor_control();
    analog_read_end(3);

    overcurrent_control();

    // Divider 10, freq = 500 Hz
    if (calib.enabled == TRUE) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }

    timer_value = (uint32)MY_TIMER_ReadCounter();
}


//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control(void) {

    static int32 pwm_input = 0;
    static uint8 motor_dir = 0;

    static int32 prev_pos;          // previous position
    static int32 prev_curr;         // previous current

    static int32 pos_error;         // position error
    static int32 curr_error;        // current error

    static int32 pos_error_sum;     // position error sum for integral
    static int32 curr_error_sum;    // current error sum for integral

    static int16 i_ref;             // current reference

    static int32 err_emg_1, err_emg_2;

    static uint8 current_emg = 0;   // 0 NONE
                                    // 1 EMG 1
                                    // 2 EMG 2
                                    // wait for both to get down

    static int32 handle_value;


    err_emg_1 = g_meas.emg[0] - c_mem.emg_threshold[0];
    err_emg_2 = g_meas.emg[1] - c_mem.emg_threshold[1];



    // =========================== POSITION INPUT ==============================
    switch(c_mem.input_mode) {

        case INPUT_MODE_ENCODER3:

            // Calculate handle value based on position of handle in the
            // sensor chain and multiplication factor between handle and motor position
            if (c_mem.double_encoder_on_off) {
                handle_value = g_meas.pos[2] * c_mem.motor_handle_ratio;
            } else {
                handle_value = g_meas.pos[1] * c_mem.motor_handle_ratio;
            }

            // Read handle and use it as reference for the motor
            if (((handle_value - g_ref.pos[0]) > c_mem.max_step_pos)   &&   (c_mem.max_step_pos != 0)) {
                g_ref.pos[0] += c_mem.max_step_pos;
            } else if (((handle_value - g_ref.pos[0]) < c_mem.max_step_neg)   &&   (c_mem.max_step_neg != 0)) {
                g_ref.pos[0] += c_mem.max_step_neg;
            } else {
                g_ref.pos[0] = handle_value;
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

    // Position limit saturation
    if (c_mem.pos_lim_flag) {
        if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) g_ref.pos[0] = c_mem.pos_lim_inf[0];
        if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) g_ref.pos[0] = c_mem.pos_lim_sup[0];
    }

    switch(c_mem.control_mode) {
        // ======================= CURRENT AND POSITION CONTROL =======================
        case CURR_AND_POS_CONTROL:
            pos_error = g_ref.pos[0] - g_meas.pos[0];

            // ------ position PID control ------

            i_ref = 0;

            // Proportional
            if (c_mem.k_p != 0) {
                i_ref += (int32)(c_mem.k_p * pos_error) >> 16;
            }

            // Integral
            if (c_mem.k_i != 0) {
                i_ref += (int32)(c_mem.k_i * pos_error_sum) >> 16;
            }

            // Derivative
            if (c_mem.k_d != 0) {
                i_ref += (int32)(c_mem.k_d * (prev_pos - g_meas.pos[0])) >> 16;
            }

            // // current set through position reference
            // i_ref = g_ref.pos[0] >> g_mem.res[0];

            // motor direction depends on i_ref
            if (i_ref >= 0) {
                motor_dir = 0x01;
            } else {
                motor_dir = 0x00;
            }

            // current ref must be positive
            i_ref = abs(i_ref);

            // saturate max current
            if (i_ref > c_mem.current_limit) {
                i_ref = c_mem.current_limit;
            }

            // saturate min current
            if (i_ref < MIN_CURR_SAT_LIMIT && i_ref > 0) {
                i_ref = MIN_CURR_SAT_LIMIT;
            }

            // // write i_ref on meas curr 2 for DEBUG
            // g_meas.curr[1] = i_ref;

            // current error
            curr_error = i_ref - g_meas.curr[0];

            // ----- current PID control -----

            pwm_input = 0;

            // Proportional
            if (c_mem.k_p_c != 0) {
                pwm_input += (int32)(c_mem.k_p_c * curr_error) >> 16;
            }

            // Integral
            if (c_mem.k_i_c != 0) {
                pwm_input += (int32)(c_mem.k_i_c * curr_error_sum) >> 16;
            }

            // Derivative
            if (c_mem.k_d_c != 0) {
                pwm_input += (int32)(c_mem.k_d_c * (prev_curr - g_meas.curr[0])) >> 16;
            }

            // pwm_input saturation
            if (pwm_input < 0) {
                pwm_input = 0;
            } else if (pwm_input > PWM_MAX_VALUE) {
                pwm_input = PWM_MAX_VALUE;
            }

            // update error sum for both errors
            pos_error_sum += pos_error;
            curr_error_sum += curr_error;

            // error_sum saturation
            if (pos_error_sum > POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum = POS_INTEGRAL_SAT_LIMIT;
            } else if (pos_error_sum < -POS_INTEGRAL_SAT_LIMIT) {
                pos_error_sum = -POS_INTEGRAL_SAT_LIMIT;
            }

            if (curr_error_sum > CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum = CURR_INTEGRAL_SAT_LIMIT;
            } else if (curr_error_sum < -CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum = -CURR_INTEGRAL_SAT_LIMIT;
            }

            // Update position
            prev_pos = g_meas.pos[0];

            // Update current
            prev_curr = g_meas.curr[0];

            break;

        // ============================== POSITION CONTROL =====================
        case CONTROL_ANGLE:
            pos_error = g_ref.pos[0] - g_meas.pos[0];

            pos_error_sum += pos_error;

            // anti-windup (for integral control)
            if (pos_error_sum > ANTI_WINDUP) {
                pos_error_sum = ANTI_WINDUP;
            } else if (pos_error_sum < -ANTI_WINDUP) {
                pos_error_sum = -ANTI_WINDUP;
            }

            // Proportional
            if (c_mem.k_p != 0) {
                pwm_input = (int32)(c_mem.k_p * pos_error) >> 16;
            }

            // Integral
            if (c_mem.k_i != 0) {
                pwm_input += (int32)(c_mem.k_i * pos_error_sum) >> 16;
            }

            // Derivative
            if (c_mem.k_d != 0) {
                pwm_input += (int32)(c_mem.k_d * (prev_pos - g_meas.pos[0])) >> 16;
            }

            // Update measure
            prev_pos = g_meas.pos[0];

            if (pwm_input > 0) {
                motor_dir = 1;
            } else {
                motor_dir = 0;
            }

            break;

        // ========================== CURRENT CONTROL ==========================
        case CONTROL_CURRENT:
            i_ref = g_ref.pos[0] >> g_mem.res[0];

            if (i_ref > 0) {
                motor_dir = 1;
            } else {
                motor_dir = 0;
            }

            // current error
            curr_error = abs(i_ref) - g_meas.curr[0];

            pwm_input = 0;

            // Proportional
            if (c_mem.k_p_c != 0) {
                pwm_input += (int32)(c_mem.k_p_c * curr_error) >> 16;
            }

            // Integral
            if (c_mem.k_i_c != 0) {
                pwm_input += (int32)(c_mem.k_i_c * curr_error_sum) >> 16;
            }

            // Derivative
            if (c_mem.k_d_c != 0) {
                pwm_input += (int32)(c_mem.k_d_c * (prev_curr - g_meas.curr[0])) >> 16;
            }

            // pwm_input saturation
            if (pwm_input < 0) {
                pwm_input = 0;
            } else if (pwm_input > PWM_MAX_VALUE) {
                pwm_input = PWM_MAX_VALUE;
            }

            // update error sum for both errors
            curr_error_sum += curr_error;

            if (curr_error_sum > CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum = CURR_INTEGRAL_SAT_LIMIT;
            } else if (curr_error_sum < -CURR_INTEGRAL_SAT_LIMIT) {
                curr_error_sum = -CURR_INTEGRAL_SAT_LIMIT;
            }

            // Update current
            prev_curr = g_meas.curr[0];

        // ================= DIRECT PWM CONTROL ================================
        case CONTROL_PWM:
            // Shift right by resolution to have the real input number
            pwm_input = g_ref.pos[0] >> g_mem.res[0];

            if (pwm_input > 0) {
                motor_dir = 1;
            } else {
                motor_dir = 0;
            }

            // pwm_input saturation
            if (pwm_input < 0) {
                pwm_input = 0;
            } else if (pwm_input > PWM_MAX_VALUE) {
                pwm_input = PWM_MAX_VALUE;
            }

            break;
    }

    ////////////////////////////////////////////////////////////////////////////

    if(pwm_input >  PWM_MAX_VALUE) pwm_input =  PWM_MAX_VALUE;
    if(pwm_input < -PWM_MAX_VALUE) pwm_input = -PWM_MAX_VALUE;


    if ((g_mem.control_mode != CONTROL_PWM) || (g_mem.control_mode != CURR_AND_POS_CONTROL)) {
        pwm_input = (((pwm_input * 1024) / PWM_MAX_VALUE) * device.pwm_limit) / 1024;
    }


    MOTOR_DIR_Write(motor_dir);
    PWM_MOTORS_WriteCompare1(abs(pwm_input));
}

//==============================================================================
//                                                               ENCODER READING
//==============================================================================

void encoder_reading(uint8 index) {

    static uint8 jj; // iterator

    static uint32 data_encoder;
    static int32 value_encoder;
    static int32 aux;

    static int32 last_value_encoder[NUM_OF_SENSORS];

    static uint8 only_first_time = 1;
    static uint8 one_time_execute = 0;

    static uint8 error[NUM_OF_SENSORS];

    int rot;

    // check index value. Eventually reset last_vale_encoder
    if (index >= NUM_OF_SENSORS)
        return;
    
    if(reset_last_value_flag) {
        for(jj=0; jj<NUM_OF_SENSORS; jj++){
            last_value_encoder[jj] = 0;
        }
        reset_last_value_flag = 0;
    }

    // Normal execution
    switch(index) {
        case 0:
            data_encoder = SHIFTREG_ENC_1_ReadData();
            break;

        case 1:
            data_encoder = SHIFTREG_ENC_2_ReadData();
            break;

        case 2:
            data_encoder = SHIFTREG_ENC_3_ReadData();
            break;
    }

    // Shift 1 right to erase Dummy bit of chain
    data_encoder = data_encoder >> 1;

    data_encoder = data_encoder & 0x3FFFF;          //0x0003FFFF reset first 14 bits

    if (check_enc_data(&data_encoder)) {

        aux = data_encoder & 0x3FFC0;           // reset last 6 bit 
                                                // -> |:|:|id|dim|CMD|CHK|(data---
        value_encoder = (aux - 0x20000) >> 2;   // subtract half of max value
                                                // and shift to have 16 bit val

        // Add offset and crop to 16bit
        value_encoder  = (int16)(value_encoder + g_mem.m_off[index]);

        // Initialize last_value_encoder
        if (only_first_time) {
            last_value_encoder[index] = value_encoder;
            if (index == 2) {
                only_first_time = 0;
            }
        }

        // take care of rotations
        aux = value_encoder - last_value_encoder[index];

        // ====================== 1 TURN ======================
        // -32768                    0                    32767 -32768                   0                     32767
        // |-------------------------|-------------------------|-------------------------|-------------------------|
        //              |                         |      |           |      |                         |
        //           -16384                     16383    |           |   -16384                     16383
        //                                               |           |
        //                                           24575           -24576
        //                                               |___________|
        //                                                   49152

        // if we are in the right interval, take care of rotation
        // and update the variable only if the difference between
        // one measure and another is less than 1/4 of turn

        // Considering we are sampling at 1kHz, this means that our shaft needs
        // to go slower than 1/4 turn every ms -> 1 turn every 4ms
        // equal to 250 turn/s -> 15000 RPM

        if (aux > 49152) {
            g_meas.rot[index]--;
        } else if (aux < -49152) {
            g_meas.rot[index]++;
        } else if (abs(aux) > 16384) { // if two measure are too far
            error[index]++;
            if (error[index] < 10) {
                // Discard
                return;
            }
            error[index] = 0;
        }

        error[index] = 0;

        last_value_encoder[index] = value_encoder;

        value_encoder += (int32)g_meas.rot[index] << 16;

        if (c_mem.m_mult[index] != 1.0) {
            value_encoder *= c_mem.m_mult[index];
        }

        g_meas.pos[index] = value_encoder;
    }

    // wait at least 3 * max_num_of_error (10) + 5 = 35 cylces to reconstruct the right turn
    if (one_time_execute < 35) {
        if (one_time_execute < 34) {
            one_time_execute++;
        } else {
            // If necessary activate motors
            g_ref.pos[0] = g_meas.pos[0];
            MOTOR_ON_OFF_Write(g_ref.onoff);

            // Double encoder translation
            if (c_mem.double_encoder_on_off) {
                rot = calc_turns_fcn(g_meas.pos[0], g_meas.pos[1]);

                if (rot > 13) {
                    g_meas.rot[0] = -27 + rot;
                } else {
                    g_meas.rot[0] = rot;
                }

            }

            one_time_execute = 35;
        }
    }
}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

void analog_read_init(uint8 index) {

    // should I execute the function for this index?
    if(index >= NUM_OF_ANALOG_INPUTS)
        return;

    AMUX_FastSelect(index);
    ADC_StartConvert();
}


void analog_read_end(uint8 index) {

    static int32 value, i_aux;

    static uint16 emg_counter_1 = 0;
    static uint16 emg_counter_2 = 0;

    static uint8 emg_1_status = 1;  // 0 normal execution
    static uint8 emg_2_status = 1;  // 1 reset status
                                    // 2 discard values
                                    // 3 sum values and mean
                                    // 4 wait

    // should I execute the function for this index?
    if(index >= NUM_OF_ANALOG_INPUTS)
        return;

    if (ADC_IsEndConversion(ADC_WAIT_FOR_RESULT)) {

        value = (int32) ADC_GetResult16();
        ADC_StopConvert();

        switch(index) {
            // --- Input tension ---
            case 0:
                device.tension = filter_v((value - 1638) * device.tension_conv_factor);
                //until there is no valid input tension repeat this measurement
                if (device.tension < 0) {
                    emg_1_status = 1;   // reset status
                    emg_2_status = 1;
                    device.tension_valid = FALSE;

                    if (c_mem.emg_calibration_flag) {
                        if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
                            g_ref.onoff = 0x00;
                            MOTOR_ON_OFF_Write(g_ref.onoff);
                        }
                    }
                } else {
                    device.tension_valid = TRUE;
                    if(g_mem.activate_pwm_rescaling)        //pwm rescaling is activated
                        pwm_limit_search();                 //only for 12V motors
                }
                break;

            // --- Current motor 1 ---
            case 1:
                if (device.tension_valid) {
                    g_meas.curr[0] =  filter_i1(abs(((value - 1638) * 4000) / (1638)));
                } else {
                    g_meas.curr[0] = 0;
                }
                break;

            // --- EMG 1 ---
            case 2:
                // execute only if there is tension
                if (device.tension_valid == FALSE) {
                    g_meas.emg[0] = 0;
                    break;
                }

                // if calibration is not needed go to "normal execution"
                if (!g_mem.emg_calibration_flag) {
                    emg_1_status = 0; //normal execution
                }
                // EMG 1 calibration state machine
                switch(emg_1_status) {
                    case 0: // normal execution
                        i_aux = filter_ch1(value);
                        i_aux = (1024 * i_aux) / g_mem.emg_max_value[0];

                        //Saturation
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
                // execute only if there is tension
                if (device.tension_valid == FALSE) {
                    g_meas.emg[1] = 0;
                    break;
                }

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
                                if (c_mem.control_mode == CONTROL_ANGLE) {
                                    g_ref.pos[0] = g_meas.pos[0];
                                    g_ref.pos[1] = g_meas.pos[1];
                                }
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
                break; // emg switch break

            default:
                break; // main switch break
        }
    }
}

//==============================================================================
//                                                           OVERCURRENT CONTROL
//==============================================================================

void overcurrent_control(void) {
    if (c_mem.current_limit != 0) {
        // if the current is over the limit
        if (g_meas.curr[0] > c_mem.current_limit) {
            //decrese pwm_limit
            device.pwm_limit--;
        // if the current is in the safe zone
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

void pwm_limit_search() {
    uint8 index;
    uint16 max_tension = 25500; 
    uint16 min_tension = 11500;

    if (device.tension > max_tension) {
        device.pwm_limit = 0;
    } else if (device.tension < min_tension) {
        device.pwm_limit = 100;
    } else {
        index = (uint8)((device.tension - min_tension) / 500);
        device.pwm_limit = pwm_preload_values[index];
    }
}
/* [] END OF FILE */