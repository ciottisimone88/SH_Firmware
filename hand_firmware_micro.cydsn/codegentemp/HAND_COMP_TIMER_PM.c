/*******************************************************************************
* File Name: HAND_COMP_TIMER_PM.c
* Version 2.70
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "HAND_COMP_TIMER.h"

static HAND_COMP_TIMER_backupStruct HAND_COMP_TIMER_backup;


/*******************************************************************************
* Function Name: HAND_COMP_TIMER_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  HAND_COMP_TIMER_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void HAND_COMP_TIMER_SaveConfig(void) 
{
    #if (!HAND_COMP_TIMER_UsingFixedFunction)
        HAND_COMP_TIMER_backup.TimerUdb = HAND_COMP_TIMER_ReadCounter();
        HAND_COMP_TIMER_backup.InterruptMaskValue = HAND_COMP_TIMER_STATUS_MASK;
        #if (HAND_COMP_TIMER_UsingHWCaptureCounter)
            HAND_COMP_TIMER_backup.TimerCaptureCounter = HAND_COMP_TIMER_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!HAND_COMP_TIMER_UDB_CONTROL_REG_REMOVED)
            HAND_COMP_TIMER_backup.TimerControlRegister = HAND_COMP_TIMER_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: HAND_COMP_TIMER_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  HAND_COMP_TIMER_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void HAND_COMP_TIMER_RestoreConfig(void) 
{   
    #if (!HAND_COMP_TIMER_UsingFixedFunction)

        HAND_COMP_TIMER_WriteCounter(HAND_COMP_TIMER_backup.TimerUdb);
        HAND_COMP_TIMER_STATUS_MASK =HAND_COMP_TIMER_backup.InterruptMaskValue;
        #if (HAND_COMP_TIMER_UsingHWCaptureCounter)
            HAND_COMP_TIMER_SetCaptureCount(HAND_COMP_TIMER_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!HAND_COMP_TIMER_UDB_CONTROL_REG_REMOVED)
            HAND_COMP_TIMER_WriteControlRegister(HAND_COMP_TIMER_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: HAND_COMP_TIMER_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  HAND_COMP_TIMER_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void HAND_COMP_TIMER_Sleep(void) 
{
    #if(!HAND_COMP_TIMER_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(HAND_COMP_TIMER_CTRL_ENABLE == (HAND_COMP_TIMER_CONTROL & HAND_COMP_TIMER_CTRL_ENABLE))
        {
            /* Timer is enabled */
            HAND_COMP_TIMER_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            HAND_COMP_TIMER_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    HAND_COMP_TIMER_Stop();
    HAND_COMP_TIMER_SaveConfig();
}


/*******************************************************************************
* Function Name: HAND_COMP_TIMER_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  HAND_COMP_TIMER_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void HAND_COMP_TIMER_Wakeup(void) 
{
    HAND_COMP_TIMER_RestoreConfig();
    #if(!HAND_COMP_TIMER_UDB_CONTROL_REG_REMOVED)
        if(HAND_COMP_TIMER_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                HAND_COMP_TIMER_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
