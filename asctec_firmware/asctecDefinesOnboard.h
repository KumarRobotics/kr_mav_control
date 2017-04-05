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

#ifndef ASCTECDEFINESONBOARD_H_
#define ASCTECDEFINESONBOARD_H_

/**
 * \defgroup vartype Variable Types
 * \brief A List of all Variable Types.
 *
 *	The varible types are needed for describing the content of a variable. The problem is, that only the size are not only describe the type of a variable fully (mostly because of signed and unsigned). Every time, if you want to get a variable, you get the type and size of the variable for finding it from the buffer.
 */

/**
 * \defgroup callbacks Callback Functions
 * \brief This is a list of all callback functions, you have to call at the beginning of your code.
 *
 * This module describes all important functions, that should be declared at the beginning of the program. If you don't do that, the fully functionality of ACI cannot be guaranteed.
 *
 */


//version defines. Minor versions work together if all info parameters match. Major versions don't work together
#define ACI_VER_MAJOR   1
#define ACI_VER_MINOR	0

//type definitions

#define LENGTH_CHAR 	1
#define LENGTH_SHORT	2
#define LENGTH_INT		4
#define LENGTH_LONGINT	8
#define LENGTH_FLOAT 	4
#define LENGTH_DOUBLE 	8

//internal defines
#define MAX_NAME_LENGTH 32
#define MAX_DESC_LENGTH 64
#define MAX_UNIT_LENGTH 32
#define MAX_VARIABLE_LIST 70
#define MAX_COMMAND_LIST 50
#define MAX_PARAMETER_LIST 50

#define MEMPACKET_MAX_VARS 64

#define ACI_RX_BUFFER_SIZE		384
#define ACI_TX_RINGBUFFER_SIZE 	160

// ID defines
#define ID_NONE 0

// Variable type defines. upper six bits define size of type in memory (0..63). Lower 2 bits are IDs from 0..3 per size
#define VARCLASS_SIGNED   0x00
#define VARCLASS_UNSIGNED 0x01
#define VARCLASS_FLOAT     0x02
#define VARCLASS_STRUCT   0x03

/**
* \ingroup vartype
* \brief 1 byte types signed
*/
#define VARTYPE_INT8 ((1<<2)|VARCLASS_SIGNED)

/**
* \ingroup vartype
* \brief 1 byte types unsigned
*/
#define VARTYPE_UINT8      ((1<<2)|VARCLASS_UNSIGNED)

/**
* \ingroup vartype
* \brief 2 byte types signed
*/
#define VARTYPE_INT16      ((2<<2)|VARCLASS_SIGNED)

/**
* \ingroup vartype
* \brief 2 byte types signed
*/
#define VARTYPE_UINT16     ((2<<2)|VARCLASS_UNSIGNED)

/**
* \ingroup vartype
* \brief 4 byte types signed
*/
#define VARTYPE_INT32      ((4<<2)|VARCLASS_SIGNED)
/**
* \ingroup vartype
* \brief 4 byte types unsigned
*/
#define VARTYPE_UINT32     ((4<<2)|VARCLASS_UNSIGNED)
/**
* \ingroup vartype
* \brief 4 byte types signed float
*/
#define VARTYPE_SINGLE     ((4<<2)|VARCLASS_FLOAT)

/**
* \ingroup vartype
* \brief 8 byte types signed
*/
#define VARTYPE_INT64      ((8<<2)|VARCLASS_SIGNED)

/**
* \ingroup vartype
* \brief 8 byte types unsigned
*/
#define VARTYPE_UINT64     ((8<<2)|VARCLASS_UNSIGNED)

/**
* \ingroup vartype
* \brief 8 byte types signed double
*/
#define VARTYPE_DOUBLE     ((8<<2)|VARCLASS_FLOAT)

/**
* \ingroup vartype
* \brief struct with size of a
*/
#define VARTYPE_STRUCT_WITH_SIZE(a) ((a<<2)|VARCLASS_STRUCT)

/**
* \ingroup vartype
* \brief 3 float vector
*/
#define VARTYPE_VECTOR_3F ((12<<2)|VARCLASS_FLOAT)
/**
* \ingroup vartype
* \brief 3 int vector
*/

#define VARTYPE_VECTOR_3I ((12<<2)|VARCLASS_SIGNED)

/**
* \ingroup vartype
* \brief quaternion (4 float) vector
*/
#define VARTYPE_QUAT ((16<<2)|VARCLASS_FLOAT)


#define NULL ((void *)0)

//RX states
#define ARS_IDLE 			0x00
#define ARS_STARTBYTE1 		0x01
#define ARS_STARTBYTE2 		0x02
#define ARS_MESSAGETYPE 	0x03
#define ARS_LENGTH1 		0x04
#define ARS_LENGTH2 		0x05
#define ARS_DATA 			0x06
#define ARS_CRC1 			0x07
#define ARS_CRC2 			0x08

//LOAD states
#define LOAD_ID1 			0x03
#define LOAD_ID2			0x04
#define LOAD_VARTYPE 		0x05
#define LOAD_DATA 			0x06

//ACI Publish param return const

#define ACI_PARAM_OK			0
#define ACI_PARAM_SIZE_TOO_BIG 	1
#define ACI_PARAM_NO_FLASH_AV	2 // No Flash avaible

// ACI Packet types
//Remote->Onboard
#define ACIMT_REQUESTVARTABLEENTRIES 		0x01
#define ACIMT_REQUESTCMDTABLEENTRIES 		0x02
#define ACIMT_GETVARTABLEINFO				0x03
#define ACIMT_GETCMDTABLEINFO				0x04

#define ACIMT_REQUESTPARAMTABLEENTRIES 		0x05
#define ACIMT_UPDATEPARAMTABLE 		  		0x06
#define ACIMT_GETPARAMTABLEINFO				0x07

#define ACIMT_UPDATEVARPACKET           	0x10
//0x10-0x1f are reserved for update var packets!
#define ACIMT_CHANGEPACKETRATE				0x20
#define ACIMT_GETPACKETRATE					0x21
#define ACIMT_HEARBEAT                  	0x22
#define ACIMT_SETHEARTBEATTIMEOUT       	0x23
#define ACIMT_GETHEARTBEATTIMEOUT       	0x24
#define ACIMT_RESETREMOTE					0x25

#define ACIMT_UPDATECMDPACKET           	0x30
//0x30-0x3f are reserved for update cmd packet config!

#define ACIMT_CMDPACKET                 	0x40
//0x40-0x4f are reserved for cmd packets!

#define ACIMT_CMDACK            	    	0x50
//0x50-0x5f are reserved for cmd packets acks!

#define ACIMT_UPDATEPARAMPACKET           	0x60
//0x60-0x6f are reserved for update param packet config!

#define ACIMT_PARAMPACKET					0x70

//Onboard->Remote
#define ACIMT_SENDVARTABLEINFO				0x80
#define ACIMT_SENDVARTABLEENTRY				0x81
#define ACIMT_SENDVARTABLEENTRYINVALID		0x82

#define ACIMT_SENDCMDTABLEINFO				0x83
#define ACIMT_SENDCMDTABLEENTRY				0x84
#define ACIMT_SENDCMDTABLEENTRYINVALID		0x85

#define ACIMT_SENDPARAMTABLEINFO			0x86
#define ACIMT_SENDPARAMTABLEENTRY			0x87
#define ACIMT_SENDPARAMTABLEENTRYINVALID	0x88
#define ACIMT_PARAM							0x89

#define ACIMT_VARPACKET                 	0x90
//0x90-0x9f are reserved for var packets!

#define ACIMT_ACK                       	0xA0
#define ACIMT_PACKETRATEINFO				0xA1
#define ACIMT_SENDHEARBEATTIMEOUT       	0xA2
#define ACIMT_SINGLESEND					0xA3
#define ACIMT_SINGLEREQ						0xA4
#define ACIMT_MAGICCODES					0xA5


//GENERAL
#define ACIMT_INFO_REQUEST					0xF0
#define ACIMT_INFO_REPLY					0xF1
#define ACIMT_SAVEPARAM						0xF2
#define ACIMT_LOADPARAM						0xF3

#define ACI_ACK_UPDATEVARPACKET         	0x10
//0x10-0x1f are reserved for ack var packets!

#define ACI_ACK_OK                          0x01
#define ACI_ACK_CRC_ERROR                   0xF0
#define ACI_ACK_PACKET_TOO_LONG				0xF1

#define ACI_DBG								0xFF

//internal structures

//this defines the maximum number of different variables packets. Both (onboard and offboard) have to be compiled with
//the setting. 3 is standard due to limited memory of the onboard code
#define MAX_VAR_PACKETS 3


#define PACKEDDEF __attribute__((packed))

struct __attribute__((packed)) ACI_FLASH_TABLE_HEADER
{
	unsigned short noOfEntries;
};

struct __attribute__((packed)) ACI_FLASH_TABLE_ENTRY
{
	unsigned short id;
	unsigned short crc;
};

struct __attribute__((packed)) ACI_MEM_TABLE_ENTRY
{
	unsigned short id;
	const char * name;
	const char * description;
	const char * unit;
	unsigned char varType;
	void * ptrToVar;

};

struct __attribute__((packed)) ACI_MEM_TABLE_ENTRY_WITH_STRINGS
{
	unsigned short id;
	char name[MAX_NAME_LENGTH];
	char description[MAX_DESC_LENGTH];
	char unit[MAX_UNIT_LENGTH];
	unsigned char varType;
	void * ptrToVar;
};

//this package is fixed and should never be changed!
struct __attribute__((packed)) ACI_INFO
{
	unsigned char verMajor;
	unsigned char verMinor;
	unsigned char maxNameLength;
	unsigned char maxDescLength;
	unsigned char maxUnitLength;
	unsigned char maxVarPackets;
	unsigned char memPacketMaxVars;
	unsigned short flags;
	unsigned short dummy[8];
};

struct __attribute__((packed)) ACI_MEM_VAR_TABLE
{
	struct ACI_MEM_TABLE_ENTRY tableEntry;
	struct ACI_MEM_VAR_TABLE * next;
	unsigned char next_exist;
};


#endif /* ASCTECDEFINESONBOARD_H_ */

// Documentation Doxygen

/**
 * \mainpage AscTec Communication Interface
 * \htmlinclude start.html
 */



