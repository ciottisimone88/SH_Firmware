// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/**
* \file         main.c
*
* \brief        Firmware main file.
* \date         June 06, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

/**
* \mainpage     Firmware
* \brief        This is the firmware of the qbHand.
* \version      6.0.0
*
* \author       _qbrobotics_
*
* \date         June 06, 2016
*
* \details      This is the firmware of the qbHand. It can control a motor and
*               read its encoder. Also can read and convert analog measurements
*               connected to the PSoC microcontroller.                
*
* \copyright    (C)  qbrobotics. All rights reserved.
*
*/


// ----------------------------------------------------------------------------
// This version changes:
//      - not reported


//=================================================================     includes

#include <device.h>
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include <interruptions.h>
#include <command_processing.h>
#include <utils.h>

//==============================================================================
//                                                                 MAIN FUNCTION
//==============================================================================

int main()
{
    // Iterator
    uint8 i;         
    
    // Variable declarations for DMA 
    
    uint8 DMA_Chan;
    uint8 DMA_TD[1];

    //================================     initializations - psoc and components

    // EEPROM

    EEPROM_Start();
    memRecall();                                        // recall configuration

    // FTDI chip enable

    CyDelay(100);
    FTDI_ENABLE_REG_Write(0x01);
    
    // RS485

    UART_RS485_Stop();                                  // stop UART
    UART_RS485_Start();                                 // start UART
    UART_RS485_Init();

    UART_RS485_ClearRxBuffer();
    UART_RS485_ClearTxBuffer();

    ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);     // RS485 isr function
    
    // WATCHDOG
    
    WATCHDOG_COUNTER_Start();
    
    ISR_WATCHDOG_StartEx(ISR_WATCHDOG_Handler);         // WATCHDOG isr function    

    // PWM

    PWM_MOTORS_Start();
    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);
    MOTOR_DIR_Write(0);
    MOTOR_ON_OFF_Write(0x00);


    // SSI encoder initializations

    COUNTER_ENC_Start();

    SHIFTREG_ENC_1_Start();
    SHIFTREG_ENC_2_Start();
    SHIFTREG_ENC_3_Start();

    // ADC

    ADC_Start();                                        // start ADC
    ADC_SOC_Write(0x01);                                // Force first read cycle
   
    // DMA
    DMA_Chan = DMA_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_TD[0] = CyDmaTdAllocate();                                                                          // Allocate TD
    CyDmaTdSetConfiguration(DMA_TD[0], 2 * 4, DMA_TD[0], TD_SWAP_EN | DMA__TD_TERMOUT_EN | TD_INC_DST_ADR); // DMA Configurations
    CyDmaTdSetAddress(DMA_TD[0], LO16((uint32)ADC_DEC_SAMP_PTR), LO16((uint32)ADC_buf));                    // Set Register Address
    CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);                                                               // Initialize Channel
    
    CyDmaChEnable(DMA_Chan, 1);                                                                             // Enable DMA

    RS485_CTS_Write(0);                                 // Clear To Send on RS485

    // TIMER

    MY_TIMER_Start();           
    PACER_TIMER_Start();

    CYGlobalIntEnable;                                  // enable interrupts

//========================================     initializations - clean variables

    RESET_COUNTERS_Write(0x00);                         // Activate encoder counters

    CyDelay(10);                                        // Wait for encoders to have a valid value

    //---------------------------------------------------  Initialize referiment structure
    for (i = NUM_OF_MOTORS; i--;) 
        g_ref.pos[i] = 0;


    if (c_mem.emg_calibration_flag) {
        if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
            (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
            (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
            (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV))
            g_ref.onoff = 0x00;
        else
            g_ref.onoff = c_mem.activ;
    } 
    else
        g_ref.onoff = c_mem.activ;
    
    //---------------------------------------------------  Initialize measurement structure
    for (i = NUM_OF_SENSORS; i--;) { 
        g_meas.pos[i] = 0;
        g_meas.rot[i] = 0;
    }

    g_refNew = g_ref;                                   // Initialize k+1 measurements structure

    g_ref.onoff = c_mem.activ;                          // Initalize Activation
    
    //---------------------------------------------------  Initialize emg structure
    g_meas.emg[0] = 0;
    g_meas.emg[1] = 0;

    MOTOR_ON_OFF_Write(g_ref.onoff);                    // Activating motors

    dev_pwm_limit = 0;                                  // Init PWM limit
    tension_valid = FALSE;                              // Init tension_valid BIT

    reset_last_value_flag = 0;

    //------------------------------------------------- Initialize package on receive from RS485
    g_rx.length = 0;
    g_rx.ready  = 0;

    //============================================================     main loop

    for(;;)
    {
        // Put the FF reset pin to LOW
        RESET_FF_Write(0x00);

        // Call function scheduler
        function_scheduler();

        //  Wait until the FF is set to 1
        while(FF_STATUS_Read() == 0){
            // On interrupt from RS485
            if (interrupt_flag){
                // Reset WDT
                WATCHDOG_REFRESH_Write(0x01);
                // Reset flags
                interrupt_flag = FALSE;
                watchdog_flag = FALSE;
                // Manage Interrupt on rs485
                interrupt_manager();
            }
            // On interrupt from WDT
            else { 
                if (watchdog_flag){
                    // Reset WDT
                    WATCHDOG_REFRESH_Write(0x01);
                    // Disactivate motors
                    g_refNew.onoff = 0x00;
                }
            }
        };

        // Command a FF reset
        RESET_FF_Write(0x01);

        // Wait for FF to be reset
        while(FF_STATUS_Read() == 1);

        if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER)
            UART_RS485_ClearRxBuffer();
    }
    return 0;
}



/* [] END OF FILE */