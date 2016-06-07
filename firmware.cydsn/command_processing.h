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

/** \name Param Getting and setting functions */
/** \{ */

//============================================================  paramSet
/** This function is used to save the value received from the user interface
 *  into the correct parameter. The correct parameter depends upon the param_type
 *  received.
 *
 * \param 	param_type 	An integer referring to the parameter to be set.
**/
void    paramSet           (uint16 param_type);

//============================================================  paramGet
/** This function is used to get the parameter value from the device memory 
 *  that will be sent through the serial interface. The selected parameter 
 *	depends upon the param_type received.
 *
 * \param 	param_type 	An integer referring to the parameter to be get.
**/void    paramGet        (uint16 param_type);

/** \} */


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


/** \name Command Processing functions */
/** \{ */

//============================================================  commProcess
/** This function unpacks the received package, depending on the command received. 	
**/
void    commProcess        ();

//============================================================  commWrite
/** This function writes on the serial port the package that needs to be sent
 * to the user.
 *
 *	\param packet_data 		The array of data that must be written.
 *	\param packet_lenght	The lenght of the data array.
 *
**/
void    commWrite          (uint8 *packet_data, uint16 packet_lenght);

/** \} */


/** \name Memory management functions */
/** \{ */

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
 * 	properly or not
 *
 *	\param value 		An ACK_OK(1) or ACK_ERROR(0) value.
**/
void    sendAcknowledgment (uint8 value);

/** \} */

#endif

/* [] END OF FILE */