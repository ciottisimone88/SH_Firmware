// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
 * \file        command_processing.h
 *
 * \brief       Received commands processing functions
 *              
 * \date         June 06, 2016
 * \author       _qbrobotics_
 * \copyright    (C)  qbrobotics. All rights reserved.
 * \details
 *
 *  This file contains all the definitions of the functions used to 
 * 	process the commands sent from the user interfaces (simulink, command line, GUI)
**/

// ----------------------------------------------------------------------------
#ifndef COMMAND_PROCESSING_H_INCLUDED
#define COMMAND_PROCESSING_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//==============================================================================
//                                                          function definitions
//==============================================================================

/** \name Firmware information functions */
/** \{ */

//============================================================  infoPrepare
/** This function is used to prepare the information string about the firmware
 *	of the device.
 *
 * \param 	info_string 	An array of chars containing firmware informations.
**/
void    infoPrepare        (unsigned char *info_string);

//============================================================  infoSend
/** This function sends the firmware information prepared with \ref infoPrepare 
 *  "infoPrepare" through the serial port to the user interface. Is used when no 
 *  ID is specified.
**/
void    infoSend           ();

//============================================================  infoGet
/** This function sends the firmware information prepared with \ref infoPrepare 
 *  "infoPrepare" through the serial port to the user interface. Is used when 
 *	the ID is specified.
 *
 * 	\param info_type	The type of the information needed.
**/
void    infoGet            (uint16 info_type);

/** \} */


/** \name Command receiving and sending functions */
/** \{ */

//============================================================  commProcess
/** This function unpacks the received package, depending on the command received. 	
**/
void    commProcess        ();

//============================================================  commWrite_old_id
/** This function writes on the serial port the package that needs to be sent
 * to the user. Is used only when a new is set, to communicate back to the APIs that
 * the new ID setting went fine or there was an error.
 *
 *	\param packet_data 		The array of data that must be written.
 *	\param packet_lenght	The lenght of the data array.
 *  \param old_id           The previous id of the board, before setting a new one
 *
**/
void    commWrite_old_id   (uint8 *packet_data, uint16 packet_lenght, uint8 old_id);

//============================================================  commWrite
/** This function writes on the serial port the package that needs to be sent
 * to the user.
 *
 *	\param packet_data 		The array of data that must be written.
 *	\param packet_lenght	The lenght of the data array.
 *  \param next             Flag that is set if the device needs to communicate with
 *                           another device on the chain.
 *
**/
void    commWrite         (uint8 *packet_data, uint16 packet_lenght, uint8 next);

/** \} */


/** \name Memory management functions */
/** \{ */

//============================================================  get_param_list
/** This function, depending on the \ref index received, gets the list of
 *  parameters with their values and sends them to user or sets a parameter
 *  from all the parameters of the device.
 *
 *	\param index 			The index of the parameters to be setted. If 0 gets
 *							full parameters list.
 *
**/
void get_param_list 		(uint16 index);

//============================================================  setZeros
/** This function sets the encoders zero position.
 *
**/
void setZeros				();

//============================================================  memStore
/** This function stores the setted parameters to the internal EEPROM memory.
 * 	It is usually called, by the user, after a parameter is set.
 *
 *	\param displacement 	The address where the parameters will be written.
 *
 *	\return A true value if the memory is correctly stored, false otherwise.
**/
uint8   memStore           (int displacement);

//============================================================  memRecall
/** This function loads user's settings from the EEPROM.
**/
void    memRecall          ();

//============================================================  memRestore
/** This function loads default settings from the EEPROM.
 * 
 *	\return A true value if the memory is correctly restored, false otherwise.
**/
uint8   memRestore         ();

//============================================================  memInit
/** This functions initializes the memory. It is used also to restore the
 * 	the parameters to their default values.
 *
 *	\return A true value if the memory is correctly initialized, false otherwise.
**/
uint8   memInit            ();

/** \} */


/** \name Utility functions */
/** \{ */

//============================================================  LCRChecksum
/** This function calculates a checksum of the array to see if the received data 
 *  is consistent.
 *
 *	\param data_array		The array of data that must be checked.	
 *	\param data_lenght		Lenght of the data array that must be checked.
 *
 *	\return The calculated checksum for the relative data_array.
**/
uint8   LCRChecksum        (uint8 *data_array, uint8 data_length);

//============================================================  sendAcknoledgment
/** This functions sends an acknowledgment to see if a command has been executed 
 * 	properly or not.
 *
 *	\param value 		An ACK_OK(1) or ACK_ERROR(0) value.
**/
void    sendAcknowledgment (uint8 value);

//==============================================================================
//                                            Service Routine interrupt function
//==============================================================================

/** \name Command processing functions */
/** \{ */

//============================================================  cmd_activate
/** This function activates the board
**/
void cmd_activate();
//============================================================  cmd_set_inputs
/** This function gets the inputs from the received package and sets them as
	motor reference.
**/
void cmd_set_inputs();
//============================================================  cmd_get_measurements
/** This function gets the encoders measurements and puts them in the package
	to be sent.
**/
void cmd_get_measurements();
//============================================================  cmd_get_measurements
/** This function gets the encoders velocities and puts them in the package
	to be sent.
**/
void cmd_get_velocities();
//============================================================  cmd_get_measurements
/** This function gets the encoders accelerations and puts them in the package
	to be sent.
**/
void cmd_get_accelerations();
//============================================================  cmd_get_currents
/** This function gets the motor current and puts it in the package to 
	be sent.
**/
void cmd_get_currents();
//============================================================  cmd_get_emg
/** This function gets the electromyographic sensors measurements and puts
	them in the package to be sent.
**/
void cmd_get_emg();
//============================================================  cmd_set_watchdog
/** This function sets the watchdog timer to the one received from the package.
	The board automatically deactivate when the time equivalent, to watchdog timer, 
	has passed.
**/
void cmd_set_watchdog();
//============================================================  cmd_get_activate
/** This function gets the board activation status and puts it in the package
	to be sent.
**/
void cmd_get_activate();
//============================================================  cmd_set_baudrate
/** This function sets the desired communication baudrate. It is possible to
	select a value equal to 460800 or 2000000.
**/
void cmd_set_baudrate();
//============================================================  cmd_get_inputs
/** This function gets the current motor reference inputs and puts them in the
	package to be sent.
**/
void cmd_get_inputs();
//============================================================  cmd_store_params
/** This function stores the parameters to the EEPROM memory
**/
void cmd_store_params();
//============================================================  cmd_ping
/** This function is used to ping the device and see if is connected.
**/
void cmd_ping();
//============================================================  cmd_get_curr_diff
/** This function is used to get the difference from the measured current and
	an estimation of it, calculated through a lookup table.
**/
void cmd_get_curr_diff();

#endif

/* [] END OF FILE */