/*

Copyright (c) 2012, Ascending Technologies GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

 */

#ifndef ASCTECCOMMINTF_H_
#define ASCTECCOMMINTF_H_

#include "asctecDefinesOnboard.h"

/** Has to be called ones during initialization and  inform aciEngine, how much calls per second it will be called.
 * @param callsPerSecond value, which told ACI, how much the aciEngine function will be called in a second (in the mainloop of the AscTec AutoPilot HL SDK are called 1000 times per second)
 * @return none
 * **/
void aciInit(unsigned short callsPerSecond);

/** handles all data processing. Has to be called a specified number of times per second. See aciSetEngineRate(); **/
void aciEngine(void);

/**
 * This functions writes all variables in the buffer, which will be send to the remote.
 */
extern void aciSyncVar(void);

/**
 * This functions writes all incoming commands in the buffer to the referenced variable, which was set in the publishing function.
 */
extern void aciSyncCmd(void);

/**
 * This functions writes all incoming parameter in the buffer to the referenced variable, which was set in the publishing function.
 */
extern void aciSyncPar(void);

/**
 * The aciReceiveHandler is fed by the UART receiving function and decodes all necessary packets.
 * @param receivedByte The received byte
 * @return none
 *
 **/
void aciReceiveHandler(unsigned char receivedByte);

/**
 * \ingroup callbacks
 * Set the callback for transmitting data over the serial device.
 */
extern void aciSetStartTxCallback(void (*aciStartTxCallback_func)(unsigned char byte)); // Callback zum SENDEN von Daten

/**
 * \ingroup callbacks
 * Set the callback for reading parameters from the EEPROM. The returned value will be send to the remote.
 */
extern void aciSetReadParafromFlashCallback(short (*aciReadParafromFlashCallback_func)(void));

/**
 * \ingroup callbacks
 * Set the callback for writing the parameters in a buffer, which will be later on written on the device. You can use this function, if you want to save the parameter on the device and write it later on the EEPROM or if you have first to fill the pages.
 */
extern void aciSetSaveParaCallback(void (*aciSaveParaCallback_func)(void));

/**
 * \ingroup callbacks
 * Set the callback for writing the buffer or the pages of the parameter on the device. The returned value will be send to the remote.
 */
extern void aciSetWriteParatoFlashCallback(short (*aciWriteParatoFlashCallback_func)(void));

/**
 * Return, if there are some bytes to transmit
 * @return true or false if there are bytes or not
 */
extern unsigned char aciTxRingBufferByteAvailable(void);

/**
 * Return the next byte out of the transmit buffer
 * @return 0 if no bytes in the buffer, otherwise the next byte
 */
extern unsigned char aciTxRingBufferGetNextByte(void);

/**
 * Send a single object.
 * @param ptr a reference to the object, which you want to send
 * @param varType type of the object (See \ref vartype)
 * @param id Any id number of the variable (not 0). That have not to be any given id number of a published variable. You need this id to identify the sent variable on your remote.
 * @param with_ack If you send it wihout any acknoledge (=0), then the variable will be sent as soon as possible. Otherwise (=1) the variable will be sent with the next aciEngine cycle, but you will be sure, that the message will be received.
 */
extern void aciSingleSend(void * ptr, unsigned char varType, unsigned short id, char with_ack);

extern int aciListParCount;
extern struct ACI_MEM_TABLE_ENTRY aciListPar[MAX_PARAMETER_LIST];

/**
 * Preparser function for publishing an ACI Variable.
 * @param var a reference to the object, which you want to publish
 * @param type type of the object (See \ref vartype)
 * @param id The id number of the object, you want to set (2 bytes)
 * @param name The name of the object (string)
 * @param description A small description of the object (string)
 * @param unit The unit of the object (string)
 */
#define aciPublishVariable(var,type,id,name,description,unit) \
	const static char cvCharName##id[]=name;\
	const static char cvCharDesc##id[]=description;\
	const static char cvCharUnit##id[]=unit;\
	aciPublishVariableInt(var,type,id,(char *)&cvCharName##id,(char *)&cvCharDesc##id,(char *)&cvCharUnit##id);

/**
 * Preparser function for publishing an ACI Command.
 * @param cmd a reference to the object, which you want to publish
 * @param type type of the object (See \ref vartype)
 * @param id The id number of the object, you want to set (2 bytes)
 * @param name The name of the object (string)
 * @param description A small description of the object (string)
 * @param unit The unit of the object (string)
 */
#define aciPublishCommand(cmd,type,id,name,description,unit) \
	const static char ccCharName##id[]=name;\
	const static char ccCharDesc##id[]=description;\
	const static char ccCharUnit##id[]=unit;\
	aciPublishCommandInt(cmd,type,id,(char *)&ccCharName##id,(char *)&ccCharDesc##id,(char *)&ccCharUnit##id);

/**
 * Preparser function for publishing an ACI Parameter.
 * @param par a reference to the object, which you want to publish
 * @param type type of the object (See \ref vartype)
 * @param id The id number of the object, you want to set (2 bytes)
 * @param name The name of the object (string)
 * @param description A small description of the object (string)
 * @param unit The unit of the object (string)
 */
#define aciPublishParameter(par,type,id,name,description,unit) \
	const static char cpCharName##id[]=name;\
	const static char cpCharDesc##id[]=description;\
	const static char cpCharUnit##id[]=unit;\
	aciPublishParameterInt(par,type,id,(char *)&cpCharName##id,(char *)&cpCharDesc##id,(char *)&cpCharUnit##id);


#endif /* ASCTECCOMMINTF_H_ */

