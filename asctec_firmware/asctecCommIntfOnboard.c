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

#include "asctecCommIntfOnboard.h"
#include <string.h>
#include <stdlib.h>

struct ACI_MEM_TABLE_ENTRY aciListVar[MAX_VARIABLE_LIST];
struct ACI_MEM_TABLE_ENTRY aciListCmd[MAX_COMMAND_LIST];
struct ACI_MEM_TABLE_ENTRY aciListPar[MAX_PARAMETER_LIST];

int aciListVarCount = 0;
int aciListCmdCount = 0;
int aciListParCount = 0;

// Variabeln
unsigned short aciVarPacketSelect[MAX_VAR_PACKETS][MEMPACKET_MAX_VARS];
unsigned char aciVarPacketSelectLength[MAX_VAR_PACKETS];
unsigned char aciVarPacketMagicCode[MAX_VAR_PACKETS]={0,0,0};
void * aciVarPacketPtrList[MAX_VAR_PACKETS][MEMPACKET_MAX_VARS];
unsigned char aciVarPacketTypeList[MAX_VAR_PACKETS][MEMPACKET_MAX_VARS];

unsigned short aciVarPacketContentBufferLength[MAX_VAR_PACKETS];
unsigned char aciVarPacketContentBuffer[MAX_VAR_PACKETS][MAX_VARIABLE_LIST*8];

unsigned short aciVarPacketTransmissionRate[MAX_VAR_PACKETS]={10,10,10};
unsigned short aciVarPacketCurrentSize[MAX_VAR_PACKETS]={0,0,0};
unsigned short aciVarPacketNumberOfVars[MAX_VAR_PACKETS]={0,0,0};
unsigned short aciVarPacketUpdated[MAX_VAR_PACKETS]={0,0,0};

// Command
unsigned short aciCmdPacketSelect[MAX_VAR_PACKETS][MEMPACKET_MAX_VARS];
unsigned char aciCmdPacketSelectLength[MAX_VAR_PACKETS];
unsigned char aciCmdPacketMagicCode[MAX_VAR_PACKETS]={0,0,0};
unsigned char aciCmdPacketWithACK[MAX_VAR_PACKETS]={0,0,0};

unsigned short aciCmdPacketContentBufferLength[MAX_VAR_PACKETS];
unsigned char aciCmdPacketContentBuffer[MAX_VAR_PACKETS][MAX_COMMAND_LIST*8];
unsigned char aciCmdPacketContentBufferValid[MAX_VAR_PACKETS];
unsigned char aciCmdPacketSendAck[MAX_VAR_PACKETS] = {0,0,0};
unsigned char aciCmdPacketReceived[MAX_VAR_PACKETS] = {0,0,0};
unsigned char aciCmdPacketContentBufferInvalidCnt[MAX_VAR_PACKETS];

// Parameter
unsigned short aciParPacketSelect[MAX_VAR_PACKETS][MEMPACKET_MAX_VARS];
unsigned char aciParPacketSelectLength[MAX_VAR_PACKETS];
unsigned char aciParPacketMagicCode[MAX_VAR_PACKETS]={0,0,0};

unsigned short aciParPacketContentBufferLength[MAX_VAR_PACKETS];
unsigned char aciParPacketContentBuffer[MAX_VAR_PACKETS][MAX_PARAMETER_LIST*8];
unsigned char aciParPacketContentBufferValid[MAX_VAR_PACKETS];
unsigned char aciParPacketSendAck[MAX_VAR_PACKETS] = {0,0,0};
unsigned char aciParamPacketReceived[MAX_VAR_PACKETS] = {0,0,0};
unsigned char aciParPacketContentBufferInvalidCnt[MAX_VAR_PACKETS];
unsigned char aciParPacketRequest[MAX_VAR_PACKETS] = {0,0,0};

unsigned char aciParamSaveIt=0;
unsigned char entry_exist = 1;

//internal global vars
unsigned int aciEngineRate=1000;
unsigned int aciEngineRateCounter[MAX_VAR_PACKETS]= {0,0,0};


//aciTxRingbuffer global vars
unsigned char aciTxRingBuffer[ACI_TX_RINGBUFFER_SIZE];
unsigned short aciTxRingBufferWritePtr=0;
unsigned char aciTxRingBufferReadPtr=0;

unsigned short aciHeartBeatCnt=0;
unsigned short aciHeartBeatTimeout=2000; // in ms. Default: 2s timeout
unsigned char aciInhibitPacketTransmission=0;

//aciTxRingbuffer prototypes
void aciTxRingBufferReset(void);
unsigned short aciTxRingBufferGetFreeSpace(void);
void aciTxRingBufferAddData(void * ptr, unsigned short size);
void aciTxSendPacket(unsigned char aciMessageType, void * data, unsigned short cnt);
void (*aciStartTxCallback)(unsigned char byte);
unsigned char (*aciReadDatafromFlashCallback)(void * ptr, unsigned short cnt);
void (*aciReadWriteStartCallback)(void) = NULL;
void (*aciReadWriteEndCallback)(void) = NULL;
void (*aciAddDatatoFlashCallback)(void * ptr, unsigned short cnt) = NULL;
short (*aciReadParafromFlashCallback)(void) = NULL;
void (*aciSaveParaCallback)(void) = NULL;
short (*aciWriteParatoFlashCallback)(void) = NULL;
void aciSendVar(void);
void aciPublishVariableInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit);
void aciPublishCommandInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit);
void aciPublishParameterInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit);
void aciSendSingleWithAck(short i);

//aciRxHandler prototypes
void aciRxHandleMessage(unsigned char messagetype, unsigned short length);


//aci helper prototypes
///update CRC with 1 byte
unsigned short aciCrcUpdate (unsigned short crc, unsigned char data);
///update crc with multiple bytes
unsigned short aciUpdateCrc16(unsigned short crc, void * data, unsigned short cnt);

// ACI Send Single
unsigned short aciSendSingleCount = 0;
unsigned short aciSendSingleNextBufferCount = 0;
unsigned short aciSendSingleStatus[MAX_VARIABLE_LIST];
unsigned short aciSendSingleId[MAX_VARIABLE_LIST];
unsigned char aciSendSingleVarType[MAX_VARIABLE_LIST];
unsigned char aciSendSingleBuffer[MAX_VARIABLE_LIST*8];
unsigned short aciSendSingleBufferCnt[MAX_VARIABLE_LIST];

unsigned short aciMagicCodeVar = 0x00FF;
unsigned short aciMagicCodeCmd = 0x00FF;
unsigned short aciMagicCodePar = 0x00FF;

void aciInit(unsigned short callsPerSecond)
{
	int i=0;
	int z=0;

	aciListVarCount = 0;
	aciListCmdCount = 0;
	aciListParCount = 0;

    for (i=0;i<MAX_VAR_PACKETS;i++)
    {
        aciCmdPacketContentBufferValid[i]=0;
        aciCmdPacketContentBufferInvalidCnt[i]=0;
        aciParPacketContentBufferValid[i]=0;
        aciParPacketContentBufferInvalidCnt[i]=0;
    	aciVarPacketContentBufferLength[i] = 0;
    }


	for (z=0;z<MAX_VAR_PACKETS;z++)
	{
		for (i=0;i<MEMPACKET_MAX_VARS;i++)
		{
			aciVarPacketSelect[z][i]=ID_NONE;
			aciVarPacketPtrList[z][i]=NULL;
			aciVarPacketTypeList[z][i]=0;
		}
		aciVarPacketMagicCode[z]=0;
		aciVarPacketSelectLength[z]=0;
		aciCmdPacketMagicCode[z]=0;
	}

	for (i=0;i<ACI_TX_RINGBUFFER_SIZE;i++) {
		aciSendSingleBuffer[i] = 0;
		aciSendSingleId[i] = 0;
		aciSendSingleVarType[i] = 0;
	}

	aciEngineRate = callsPerSecond;

}

void aciSendSingleWithAck(short i)
{
	short notzero_i = i+1;
	unsigned char varType = aciSendSingleVarType[i];
	unsigned char buffer[5+(varType>>2)];

	memcpy(&buffer[0],&notzero_i,2);
	memcpy(&buffer[2],&aciSendSingleId[i],2);
	memcpy(&buffer[4],&varType,1);
	memcpy(&buffer[5],&aciSendSingleBuffer[aciSendSingleBufferCnt[i]],varType>>2);
	aciSendSingleStatus[i]++;

	aciTxSendPacket(ACIMT_SINGLESEND,buffer,5+(varType>>2));

}

void aciSingleSend(void * ptr, unsigned char varType, unsigned short id, char with_ack) {
	if(!varType || !id) return;
	if((aciSendSingleNextBufferCount+(varType>>2))>MAX_VARIABLE_LIST*8) return;

	unsigned char buffer[5+(varType>>2)];
	int i;
	int use_i = -1;
	short zero = 0;

	if(with_ack) {
		for(i=0;i<MAX_VARIABLE_LIST;i++) {
			if((use_i==-1) && (aciSendSingleStatus[i]==0)) use_i=i;
		}
		if(use_i==-1) return; // security-if

		aciSendSingleStatus[use_i] = 1;
		aciSendSingleId[use_i] = id;
		aciSendSingleVarType[use_i] = varType;
		memcpy(&aciSendSingleBuffer[aciSendSingleNextBufferCount],ptr,varType>>2);
		aciSendSingleBufferCnt[use_i]=aciSendSingleNextBufferCount;
		aciSendSingleNextBufferCount+=varType>>2;
		aciSendSingleCount++;
	} else {
		memcpy(&buffer[0],&zero,2);
		memcpy(&buffer[2],&id,2);
		memcpy(&buffer[4],&varType,1);
		memcpy(&buffer[5],ptr,varType>>2);
		aciTxSendPacket(ACIMT_SINGLESEND,buffer,5+(varType>>2));
	}
}

/* published Variable. Variable has to be global! **/
void aciPublishVariableInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit) {
	if (aciListVarCount==MAX_VARIABLE_LIST) return;

	aciListVar[aciListVarCount].id= id;
	aciListVar[aciListVarCount].varType = varType;
	aciListVar[aciListVarCount].name = name;
	aciListVar[aciListVarCount].description = description;
	aciListVar[aciListVarCount].unit = unit;
	aciListVar[aciListVarCount].ptrToVar = (void*) ptr;

	aciMagicCodeVar = aciUpdateCrc16(aciMagicCodeVar,&id,2);
	aciMagicCodeVar = aciUpdateCrc16(aciMagicCodeVar,&varType,1);

	aciListVarCount++;
}


/* published Command. Command has to be global! **/
void aciPublishCommandInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit)
{
	if (aciListCmdCount==MAX_PARAMETER_LIST) return;

	aciListCmd[aciListCmdCount].id= id;
	aciListCmd[aciListCmdCount].varType = varType;
	aciListCmd[aciListCmdCount].name = name;
	aciListCmd[aciListCmdCount].description = description;
	aciListCmd[aciListCmdCount].unit = unit;
	aciListCmd[aciListCmdCount].ptrToVar = (void*) ptr;

	aciMagicCodeCmd = aciUpdateCrc16(aciMagicCodeCmd,&id,2);
	aciMagicCodeCmd = aciUpdateCrc16(aciMagicCodeCmd,&varType,1);

	aciListCmdCount++;
}

/** published Parameter. Parameter has to be global! **/
void aciPublishParameterInt(void * ptr, unsigned char varType, unsigned short id, char * name, char * description, char * unit)
{
	if (aciListParCount==MAX_PARAMETER_LIST) return;

	aciListPar[aciListParCount].id= id;
	aciListPar[aciListParCount].varType = varType;
	aciListPar[aciListParCount].name = name;
	aciListPar[aciListParCount].description = description;
	aciListPar[aciListParCount].unit = unit;
	aciListPar[aciListParCount].ptrToVar = (void*) ptr;

	aciMagicCodePar = aciUpdateCrc16(aciMagicCodePar,&id,2);
	aciMagicCodePar = aciUpdateCrc16(aciMagicCodePar,&varType,1);

	aciListParCount++;
}

void aciSendVar(void)
{
	short i=0;
	short j=0;
	short ii=0;
	short z = 0;
	 for (i=0;i<MAX_VAR_PACKETS;i++)
//	short i=2;
	 {

		//handle variable packet generation and triggering
		if (!aciVarPacketTransmissionRate[i]) {
		//	continue;
		}
		else  {
			unsigned short packetSize;

			//generate packet and send it

			//if packet was changed, update pointer table and size
			if (aciVarPacketUpdated[i]) {

				unsigned short currentPos = 0;

				//reset update flag
				aciVarPacketUpdated[i] = 0;

				for (j=0;j<aciVarPacketSelectLength[i];j++)
				{

					unsigned short id;

					id=aciVarPacketSelect[i][j];

					for(ii=0;ii<aciListVarCount;ii++) {
						if (aciListVar[ii].id==id)
						{
							aciVarPacketPtrList[i][currentPos] = aciListVar[ii].ptrToVar;
							aciVarPacketTypeList[i][currentPos++] =	aciListVar[ii].varType;
							break;
						}
					}
				}
				aciVarPacketNumberOfVars[i] = currentPos;

				//get packet size
				packetSize = 0;
				for (z = 0; z < MEMPACKET_MAX_VARS; z++) {
					if (aciVarPacketPtrList[i][z] == NULL
						)
						break;
					packetSize += aciVarPacketTypeList[i][z] >> 2;
				}
				aciVarPacketCurrentSize[i] = packetSize;
				aciVarPacketContentBufferLength[i] = packetSize;
			}

			//check for free space in ring buffer
			else if ((aciVarPacketNumberOfVars[i])&&(aciVarPacketCurrentSize[i] + 10 < aciTxRingBufferGetFreeSpace())&&(!aciInhibitPacketTransmission)&&(aciVarPacketTransmissionRate[i]<aciEngineRateCounter[i])) {

				unsigned char startstring[3] = { '!', '#', '!' };
				unsigned char messageType = ACIMT_VARPACKET+i;
				unsigned short crc = 0xFF;
				unsigned short psize=aciVarPacketCurrentSize[i]+1;

				aciEngineRateCounter[i]=1;

				//add header to ringbuffer
				aciTxRingBufferAddData(&startstring, 3);

				//add message type to ringbuffer
				aciTxRingBufferAddData(&messageType, 1);
				crc=aciUpdateCrc16(crc,&messageType,1);


				//add data size to ringbuffer
				aciTxRingBufferAddData(&psize, 2);
				crc=aciUpdateCrc16(crc,&psize,2);


				aciTxRingBufferAddData(&aciVarPacketMagicCode[i],1);
				crc=aciUpdateCrc16(crc,&aciVarPacketMagicCode[i],1);

				aciTxRingBufferAddData(aciVarPacketContentBuffer[i],aciVarPacketContentBufferLength[i]);
				crc=aciUpdateCrc16(crc,&aciVarPacketContentBuffer[i][0],aciVarPacketContentBufferLength[i]);

				//add CRC to ringbuffer
				aciTxRingBufferAddData(&crc, 2);

			}
		}
	 }
}

void aciSyncVar(void) {
	short i = 0;
	int z = 0;
	for (i = 0; i < MAX_VAR_PACKETS; i++)
		if (aciVarPacketNumberOfVars[i]) {
			short cnt = 0;
			for (z = 0; z < aciVarPacketNumberOfVars[i]; z++) {
				memcpy(&aciVarPacketContentBuffer[i][cnt], aciVarPacketPtrList[i][z], aciVarPacketTypeList[i][z] >> 2);
				cnt += aciVarPacketTypeList[i][z] >> 2;

			}
		}
}

void aciSyncCmd(void) {

	short j = 0;
	short i = 0;
	short k = 0;
	for (j = 0; j < MAX_VAR_PACKETS; j++)
		if (aciCmdPacketReceived[j]) {
			short cnt = 0;
			for (i = 0; i < aciCmdPacketSelectLength[j]; i++) {
				for (k =0; k < aciListCmdCount;k++)  {
					if (aciListCmd[k].id == aciCmdPacketSelect[j][i]) {
						memcpy(aciListCmd[k].ptrToVar, &aciCmdPacketContentBuffer[j][cnt], aciListCmd[k].varType >> 2);
						cnt += aciListCmd[k].varType >> 2;
						break;
					}
				}
			}
			aciCmdPacketReceived[j]=0;
		}
}

void aciSyncPar(void) {

	short j = 0;
	short i = 0;
	short k = 0;
	for (j = 0; j < MAX_VAR_PACKETS; j++)
		if (aciParamPacketReceived[j]) {
			short cnt = 0;
			for (i = 0; i < aciParPacketSelectLength[j]; i++) {
				for (k =0; k < aciListParCount;k++)  {
					if (aciListPar[k].id == aciParPacketSelect[j][i]) {
						memcpy(aciListPar[k].ptrToVar, &aciParPacketContentBuffer[j][cnt], aciListPar[k].varType >> 2);
						cnt += aciListPar[k].varType >> 2;
						break;
					}
				}
			}
			aciParamPacketReceived[j]=0;
		}

}

/** handles all data processing. Has to be called a specified number of times per second. See aciSetEngineRate(); **/
void aciEngine(void)
{
	int i=0;

	//handle heart beat

	aciHeartBeatCnt++;


	if (aciHeartBeatCnt>=aciHeartBeatTimeout)
	{
		//stop transmitting packets
		aciHeartBeatCnt=0;
		aciInhibitPacketTransmission=1;
	}

	if (!aciInhibitPacketTransmission)
	{
		aciSendVar();
	}

	for(i=0;i<MAX_VAR_PACKETS;i++)
		{
		aciEngineRateCounter[i]++;
		if(aciEngineRateCounter[i]>(5*aciEngineRate)) aciEngineRateCounter[i]=0;
		}

	if (aciParamSaveIt && (aciWriteParatoFlashCallback)) {
		short output;
		output = aciWriteParatoFlashCallback();
		aciParamSaveIt = 0;
		aciTxSendPacket(ACIMT_SAVEPARAM,&output,2);
	}

	if(aciSendSingleCount) {
//		unsigned short temp_singleCount = 0;

		for(i=0;i<MAX_VARIABLE_LIST;i++) {
			if(aciSendSingleStatus[i]==0) continue;
			else if(aciSendSingleStatus[i]==1) {
				aciSendSingleWithAck(i);
			} else {
				if(aciSendSingleStatus[i]==(aciEngineRate+2)) {
					aciSendSingleStatus[i]=1;
				} else aciSendSingleStatus[i]++;
			}
		}
	} else aciSendSingleNextBufferCount=0;
}

/*
 * ACI RX Part
 *
 *
 *
 */

unsigned char aciRxDataBuffer[ACI_RX_BUFFER_SIZE];
unsigned short aciRxDataCnt;

void aciRxHandleMessage(unsigned char messagetype, unsigned short length)
{
	short i=0;
	int z=0;
	int k=0;

	unsigned char packetSelect;
	short i_single ;
	struct ACI_INFO aciInfo;
	unsigned char c[2];
	unsigned char switch_type;
	short output;
	if((messagetype>ACIMT_UPDATEVARPACKET) && (messagetype<ACIMT_UPDATEVARPACKET+0x0f)) switch_type = ACIMT_UPDATEVARPACKET;
	else if((messagetype>ACIMT_UPDATECMDPACKET) && (messagetype<ACIMT_UPDATECMDPACKET+0x0f)) switch_type = ACIMT_UPDATECMDPACKET;
	else if((messagetype>ACIMT_UPDATEPARAMPACKET) &&(messagetype<ACIMT_UPDATEPARAMPACKET+0x0f)) switch_type = ACIMT_UPDATEPARAMPACKET;
	else if((messagetype>ACIMT_CMDPACKET) && (messagetype<ACIMT_CMDPACKET+0x0f)) switch_type = ACIMT_CMDPACKET;
	else if((messagetype>ACIMT_PARAMPACKET) && (messagetype<ACIMT_PARAMPACKET+0x0f)) switch_type = ACIMT_PARAMPACKET;
	else switch_type=messagetype;

	switch (switch_type)
	{
	case ACIMT_INFO_REQUEST:
		aciInfo.verMajor=ACI_VER_MAJOR;
		aciInfo.verMinor=ACI_VER_MINOR;
		aciInfo.maxDescLength=MAX_DESC_LENGTH;
		aciInfo.maxNameLength=MAX_NAME_LENGTH;
		aciInfo.maxUnitLength=MAX_UNIT_LENGTH;
		aciInfo.maxVarPackets=MAX_VAR_PACKETS;
		aciInfo.memPacketMaxVars=MEMPACKET_MAX_VARS;
		aciInfo.flags=0;
		for (i=0;i<8;i++)
			aciInfo.dummy[i]=0;
		aciTxSendPacket(ACIMT_INFO_REPLY,&aciInfo,sizeof(aciInfo));

	break;
	case ACIMT_REQUESTVARTABLEENTRIES:
		for (i=0;i<length/2;i++)
		{
			unsigned short selectedId;
			unsigned char idFound=0;

			selectedId=(aciRxDataBuffer[i*2+1]<<8)|aciRxDataBuffer[i*2];
			//determine no of entries in var table
			for(k=0;k<aciListVarCount;k++) {
				if (selectedId==aciListVar[k].id)
				{
					idFound=1;
					break;
				}
			}
			if ((idFound))
				{
					//before we send the data, the strings have to be copied from flash to memory
					struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS tableEntry;

					if (strlen(aciListVar[k].name)>MAX_NAME_LENGTH)
					{
						memcpy(&(tableEntry.name[0]),aciListVar[k].name,MAX_NAME_LENGTH);
						tableEntry.name[MAX_NAME_LENGTH-1]=0;
					}else
						strcpy(&(tableEntry.name[0]),aciListVar[k].name);

					if (strlen(aciListVar[k].description)>MAX_DESC_LENGTH)
					{
						memcpy(&(tableEntry.description[0]),aciListVar[k].description,MAX_NAME_LENGTH);
						tableEntry.description[MAX_DESC_LENGTH-1]=0;
					}else
						strcpy(&(tableEntry.description[0]),aciListVar[k].description);

					if (strlen(aciListVar[k].unit)>MAX_UNIT_LENGTH)
					{
						memcpy(&(tableEntry.unit[0]),aciListVar[k].unit,MAX_UNIT_LENGTH);
						tableEntry.unit[MAX_UNIT_LENGTH-1]=0;
					}else
						strcpy(&(tableEntry.unit[0]),aciListVar[k].unit);

					tableEntry.id=aciListVar[k].id;
					tableEntry.varType=aciListVar[k].varType;
					tableEntry.ptrToVar=NULL;
					aciTxSendPacket(ACIMT_SENDVARTABLEENTRY,&tableEntry,sizeof (struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS)-sizeof(tableEntry.ptrToVar));

				}else
					aciTxSendPacket(ACIMT_SENDVARTABLEENTRYINVALID,&selectedId,2);

		}

	break;
	case ACIMT_GETVARTABLEINFO:
	{
		unsigned short idList[aciListVarCount*2+2];
		unsigned short cnt=1;

		idList[0]=(unsigned short)aciListVarCount;

		for(k=0;k<aciListVarCount;k++)
				idList[cnt++]=aciListVar[k].id;

		//send no of entries
		aciTxSendPacket(ACIMT_SENDVARTABLEINFO,&idList[0],aciListVarCount*2+2);
	}
	break;
	case ACIMT_UPDATEVARPACKET:
			packetSelect=messagetype-ACIMT_UPDATEVARPACKET;
			aciInhibitPacketTransmission=0;
			if (packetSelect>=MAX_VAR_PACKETS)
				break;
			if (length>MEMPACKET_MAX_VARS*2+1)
			{
				c[0]=ACIMT_UPDATEVARPACKET+packetSelect;
				c[1]=ACI_ACK_PACKET_TOO_LONG;
				aciTxSendPacket(ACIMT_ACK,&c[0],2);
				break;
			}

			for (i=0;i<MEMPACKET_MAX_VARS;i++)
			{
				aciVarPacketSelect[packetSelect][i]=ID_NONE;
				aciVarPacketPtrList[packetSelect][i]=NULL;
				aciVarPacketTypeList[packetSelect][i]=0;
			}
			aciVarPacketMagicCode[packetSelect]=aciRxDataBuffer[0];
			memcpy(&aciVarPacketSelect[packetSelect],&aciRxDataBuffer[1],length-1);
			aciVarPacketSelectLength[packetSelect]=(length-1)/2;
			aciVarPacketContentBufferLength[packetSelect] = aciVarPacketSelectLength[packetSelect];
			aciVarPacketUpdated[packetSelect]=1;
			c[0]=ACIMT_UPDATEVARPACKET+packetSelect;
			c[1]=ACI_ACK_OK;
			aciTxSendPacket(ACIMT_ACK,&c[0],2);

		break;

	case ACIMT_REQUESTCMDTABLEENTRIES:
		for (i=0;i<length/2;i++)
		{
			unsigned short selectedId;
			unsigned char  idFound=0;

			selectedId=(aciRxDataBuffer[i*2+1]<<8)|aciRxDataBuffer[i*2];
			//determine no of entries in var table
			for(k=0;k<aciListCmdCount;k++) {
					if (selectedId==aciListCmd[k].id)
					{
						idFound=1;
						break;
					}
			}
			if ((idFound))
				{
					//before we send the data, the strings have to be copied from flash to memory
					struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS tableEntry;

					if (strlen(aciListCmd[k].name)>MAX_NAME_LENGTH)
					{
						memcpy(&(tableEntry.name[0]),aciListCmd[k].name,MAX_NAME_LENGTH);
						tableEntry.name[MAX_NAME_LENGTH-1]=0;
					} else
						strcpy(&(tableEntry.name[0]),aciListCmd[k].name);

					if (strlen(aciListCmd[k].description)>MAX_DESC_LENGTH)
					{
						memcpy(&(tableEntry.description[0]),aciListCmd[k].description,MAX_NAME_LENGTH);
						tableEntry.description[MAX_DESC_LENGTH-1]=0;
					}else
						strcpy(&(tableEntry.description[0]),aciListCmd[k].description);

					if (strlen(aciListCmd[k].unit)>MAX_UNIT_LENGTH)
					{
						memcpy(&(tableEntry.unit[0]),aciListCmd[k].unit,MAX_UNIT_LENGTH);
						tableEntry.unit[MAX_UNIT_LENGTH-1]=0;
					}else
						strcpy(&(tableEntry.unit[0]),aciListCmd[k].unit);

					tableEntry.id=aciListCmd[k].id;
					tableEntry.varType=aciListCmd[k].varType;
					tableEntry.ptrToVar=NULL;
					aciTxSendPacket(ACIMT_SENDCMDTABLEENTRY,&tableEntry,sizeof (struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS)-sizeof(tableEntry.ptrToVar));

				}else
					aciTxSendPacket(ACIMT_SENDCMDTABLEENTRYINVALID,&selectedId,2);

		}

	break;
	case ACIMT_GETCMDTABLEINFO:
	{
		unsigned short idList[aciListCmdCount*2+2];
		unsigned short cnt=1;

		idList[0]=aciListCmdCount;

		for(k=0;k<aciListCmdCount;k++) {
			idList[cnt++]=aciListCmd[k].id;
		}
		//send no of entries
		aciTxSendPacket(ACIMT_SENDCMDTABLEINFO,&idList[0],aciListCmdCount*2+2);
	}
	break;
	case ACIMT_UPDATECMDPACKET:
			packetSelect=messagetype-ACIMT_UPDATECMDPACKET;
			if (packetSelect>=MAX_VAR_PACKETS)
				break;
			if (length>MEMPACKET_MAX_VARS*2+1)
			{
				c[0]=ACIMT_UPDATECMDPACKET+packetSelect;
				c[1]=ACI_ACK_PACKET_TOO_LONG;
				aciTxSendPacket(ACIMT_ACK,&c[0],2);
				break;
			}

			aciCmdPacketMagicCode[packetSelect]=aciRxDataBuffer[0];
			aciCmdPacketWithACK[packetSelect]=aciRxDataBuffer[1];
			memcpy(&aciCmdPacketSelect[packetSelect], &aciRxDataBuffer[2],length - 2);
			short cnta = 0;
			for (i = 0; i < (length - 2) / 2; i++) {
				for(k=0;k<aciListCmdCount;k++) {
					if (aciListCmd[k].id
							== aciCmdPacketSelect[packetSelect][i]) {
						cnta += aciListCmd[k].varType >> 2;
						break;
					}
				}

			}
	     	if(cnta==0) break;
			aciCmdPacketContentBufferLength[packetSelect] = cnta;
			aciCmdPacketSelectLength[packetSelect]=(length-2)/2;
			c[0]=ACIMT_UPDATECMDPACKET+packetSelect;
			c[1]=ACI_ACK_OK;
			aciTxSendPacket(ACIMT_ACK,&c[0],2);

		break;
	case ACIMT_UPDATEPARAMPACKET:

				packetSelect=messagetype-ACIMT_UPDATEPARAMPACKET;
				unsigned char temp_buffer[MEMPACKET_MAX_VARS*16];
				if (packetSelect>=MAX_VAR_PACKETS)
					break;
				if (length>MEMPACKET_MAX_VARS*2+1)
				{
					c[0]=ACIMT_UPDATEPARAMPACKET+packetSelect;
					c[1]=ACI_ACK_PACKET_TOO_LONG;
					aciTxSendPacket(ACIMT_ACK,&c[0],2);
					break;
				}

				aciParPacketMagicCode[packetSelect]=aciRxDataBuffer[0];
				memcpy(&aciParPacketSelect[packetSelect],&aciRxDataBuffer[1],length-1);

				temp_buffer[0]=ACIMT_UPDATEPARAMPACKET+packetSelect;
				temp_buffer[1]=ACI_ACK_OK;
				short cunt = 0;
				for (i = 0; i < (length - 1) / 2; i++) {
					for (k=0;k<aciListParCount;k++) {
						if (aciListPar[k].id == aciParPacketSelect[packetSelect][i]) {
							memcpy(&temp_buffer[cunt+2],aciListPar[k].ptrToVar,aciListPar[k].varType >> 2);
							cunt += aciListPar[k].varType >> 2;
							break;
						}
					}
				}
				aciParPacketContentBufferLength[packetSelect]=cunt;
				aciParPacketSelectLength[packetSelect]=(length-1)/2;
//				c[0]=ACIMT_UPDATEPARAMPACKET+packetSelect;
//				c[1]=ACI_ACK_OK;
//				aciTxSendPacket(ACI_DBG,&temp_buffer[2],2);
//				aciTxSendPacket(ACI_DBG,&temp_buffer[4],2);
				aciTxSendPacket(ACIMT_ACK,&temp_buffer[0],2+cunt);

			break;

    case ACIMT_CMDPACKET:
         //check magic code to see that the packet fits the desired configuration

         packetSelect=messagetype-ACIMT_CMDPACKET;
//     	aciTxSendPacket(ACI_DBG, &aciCmdPacketContentBufferLength[packetSelect], 2);
//     	aciTxSendPacket(ACI_DBG, &length, 2);
         if (packetSelect>=MAX_VAR_PACKETS)
            break;
         if (((aciCmdPacketContentBufferLength[packetSelect]==length-1))) //aciCmdPacketMagicCode[packetSelect]==aciRxDataBuffer[0]) &&
         {
        	memcpy(&aciCmdPacketContentBuffer[packetSelect][0],&aciRxDataBuffer[1],length-1);
        	aciCmdPacketReceived[packetSelect]=1;
			if (aciCmdPacketWithACK[packetSelect]) {
				c[0] = ACIMT_CMDACK + packetSelect;
				aciTxSendPacket(ACIMT_ACK, &c[0], 1);
			}
			aciCmdPacketContentBufferValid[packetSelect] = 1;
			aciCmdPacketContentBufferInvalidCnt[packetSelect] = 0;

		} else {
			aciCmdPacketContentBufferValid[packetSelect] = 0;
			aciCmdPacketContentBufferInvalidCnt[packetSelect]++;
         }
    break;

    // Parameter
    case ACIMT_REQUESTPARAMTABLEENTRIES:
    		for (i=0;i<length/2;i++)
    		{
    			unsigned short selectedId;
    			unsigned char  idFound=0;
    			k=0;

    			selectedId=(aciRxDataBuffer[i*2+1]<<8)|aciRxDataBuffer[i*2];
    			//determine no of entries in param table
    			for(k=0;k<aciListParCount;k++) {
    							if (selectedId==aciListPar[k].id)
    							{
    								idFound=1;
    								break;
    							}
    						}
    			if ((idFound))
    				{
    					//before we send the data, the strings have to be copied from flash to memory
    					struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS tableEntry;

    					if (strlen(aciListPar[k].name)>MAX_NAME_LENGTH)
    					{
    						memcpy(&(tableEntry.name[0]),aciListPar[k].name,MAX_NAME_LENGTH);
    						tableEntry.name[MAX_NAME_LENGTH-1]=0;
    					} else
    						strcpy(&(tableEntry.name[0]),aciListPar[k].name);

    					if (strlen(aciListPar[k].description)>MAX_DESC_LENGTH)
    					{
    						memcpy(&(tableEntry.description[0]),aciListPar[k].description,MAX_NAME_LENGTH);
    						tableEntry.description[MAX_DESC_LENGTH-1]=0;
    					} else
    						strcpy(&(tableEntry.description[0]),aciListPar[k].description);

    					if (strlen(aciListPar[k].unit)>MAX_UNIT_LENGTH)
    					{
    						memcpy(&(tableEntry.unit[0]),aciListPar[k].unit,MAX_UNIT_LENGTH);
    						tableEntry.unit[MAX_UNIT_LENGTH-1]=0;
    					}else
    						strcpy(&(tableEntry.unit[0]),aciListPar[k].unit);

    					tableEntry.id=aciListPar[k].id;
    					tableEntry.varType=aciListPar[k].varType;
    					tableEntry.ptrToVar=NULL;
    					aciTxSendPacket(ACIMT_SENDPARAMTABLEENTRY,&tableEntry,sizeof (struct ACI_MEM_TABLE_ENTRY_WITH_STRINGS)-sizeof(tableEntry.ptrToVar));

    				}else
    					aciTxSendPacket(ACIMT_SENDPARAMTABLEENTRYINVALID,&selectedId,2);

    		}

    	break;
    	case ACIMT_GETPARAMTABLEINFO:
    	{
    		unsigned short idList[aciListParCount*2+2];
    		unsigned short cnt=1;
    		k = 0;

    		//determine no of entries in cmd table
    		idList[0]=aciListParCount;

    		for(k=0;k<aciListParCount;k++)  {
    			idList[cnt++]=aciListPar[k].id;
    		}

    		//send no of entries
    		aciTxSendPacket(ACIMT_SENDPARAMTABLEINFO,&idList[0],aciListParCount*2+2);

    	}
    	break;
    	case ACIMT_PARAM:
    		/* FOR ONLY ONE PARAMETER ID */
		if (length == 2) {
			unsigned short id;
			unsigned char sendbuffer[30];
			unsigned char newlen = 0;
			k = 0;
			id = (aciRxDataBuffer[1] << 8) | aciRxDataBuffer[0];
			//determine no of entries in cmd table
			for(k=0;k<aciListParCount;k++) {
				if(aciListPar[k].id==id)
				{
					newlen=(aciListPar[k].varType>>2)+2;
					memcpy(&sendbuffer[0], &id, 2);
					memcpy(&sendbuffer[2], aciListPar[k].ptrToVar, newlen-2);
					break;
				}
			}

			if (newlen==0) {
				sendbuffer[0]=0x00;
				aciTxSendPacket(ACIMT_PARAM, &sendbuffer[0], 1);
				break;
			}
			aciTxSendPacket(ACIMT_PARAM, &sendbuffer[0], newlen);
		}
		break;


    	case ACIMT_PARAMPACKET:
    		//check magic code to see that the packet fits the desired configuration
    		packetSelect=messagetype-ACIMT_PARAMPACKET;
    		if (packetSelect>=MAX_VAR_PACKETS)
    			break;
    		if ((aciParPacketMagicCode[packetSelect]==aciRxDataBuffer[0])) // && (aciParPacketContentBufferLength[packetSelect]==length-1))
    		{
    			memcpy(&aciParPacketContentBuffer[packetSelect][0],&aciRxDataBuffer[1],length-1);
    			aciParamPacketReceived[packetSelect]=1;
    			c[0]=ACIMT_PARAMPACKET+packetSelect;
    			aciTxSendPacket(ACIMT_ACK,&c[0],1);
    			aciParPacketContentBufferValid[packetSelect] = 1;
    			aciParPacketContentBufferInvalidCnt[packetSelect] = 0;
    		}
    		else {
    			aciParPacketContentBufferValid[packetSelect] = 0;
    			aciParPacketContentBufferInvalidCnt[packetSelect]++;
    		}
    		break;

	case ACIMT_CHANGEPACKETRATE:
		aciInhibitPacketTransmission=0;

		if (length==sizeof(aciVarPacketTransmissionRate))
		{
			for(i=0;i<(sizeof(aciVarPacketTransmissionRate)/2);i++) {
				aciVarPacketTransmissionRate[i]=(aciRxDataBuffer[i*2+1]<<8) | (aciRxDataBuffer[i*2]);
			}
		}
	break;
	case ACIMT_GETPACKETRATE:
		aciTxSendPacket(ACIMT_PACKETRATEINFO,&aciVarPacketTransmissionRate[0],sizeof(aciVarPacketTransmissionRate));
	break;
	case ACIMT_HEARBEAT:
		aciInhibitPacketTransmission=0;
		aciHeartBeatCnt=0;
	break;
	case ACIMT_GETHEARTBEATTIMEOUT:
		aciTxSendPacket(ACIMT_SENDHEARBEATTIMEOUT,&aciHeartBeatTimeout,2);
	break;
	case ACIMT_SETHEARTBEATTIMEOUT:
		if (length==2)
			memcpy(&aciHeartBeatTimeout,&aciRxDataBuffer[0],2);
	break;
	case ACIMT_MAGICCODES:
		{
			unsigned char temp_buffer[12];
			memcpy(&temp_buffer[0], &aciMagicCodeVar,2);
			memcpy(&temp_buffer[2], &aciMagicCodeCmd,2);
			memcpy(&temp_buffer[4], &aciMagicCodePar,2);
			memcpy(&temp_buffer[6], &aciListVarCount,2);
			memcpy(&temp_buffer[8], &aciListCmdCount,2);
			memcpy(&temp_buffer[10], &aciListParCount,2);
			aciTxSendPacket(ACIMT_MAGICCODES,&temp_buffer[0],12);
		}
		break;
	case ACIMT_RESETREMOTE:
		//reset var packet content
		for (z=0;z<MAX_VAR_PACKETS;z++)
			{
				for (i=0;i<MEMPACKET_MAX_VARS;i++)
				{
					aciVarPacketSelect[z][i]=ID_NONE;
					aciVarPacketPtrList[z][i]=NULL;
					aciVarPacketTypeList[z][i]=0;
				}
				aciVarPacketMagicCode[z]=0;
				aciVarPacketSelectLength[z]=0;
			}
		aciInhibitPacketTransmission=0;
		aciHeartBeatCnt=0;

	break;
	case ACIMT_SAVEPARAM:
		if (aciSaveParaCallback) {
			aciSaveParaCallback();
			aciParamSaveIt = 1;
		}
		break;

	case ACIMT_LOADPARAM:
		if (aciReadParafromFlashCallback) {
			output = aciReadParafromFlashCallback();
			aciTxSendPacket(ACIMT_LOADPARAM, &output, 2);
		}
		break;

	case ACIMT_SINGLESEND: // ACK
		//if(length>0) {
			i_single = (aciRxDataBuffer[1]<<8)|aciRxDataBuffer[0];
			i_single=i_single-1;
			if(i<MAX_VARIABLE_LIST) aciSendSingleStatus[i_single]=0;
		//}
		break;

	case ACIMT_SINGLEREQ:
		if(length>1) {
			char idFound=0;
			unsigned char temp_buffer[512];

			i_single = (aciRxDataBuffer[1]<<8)|aciRxDataBuffer[0];
			for(k=0;k<aciListVarCount;k++) {
				if (i_single==aciListVar[k].id)
				{
					idFound=1;
					break;
				}
			}
//
			if (idFound)
			{
				memcpy(&temp_buffer[0],&(aciListVar[k].id),2);
				memcpy(&temp_buffer[2],&(aciListVar[k].varType),1);
				memcpy(&temp_buffer[3],aciListVar[k].ptrToVar, (aciListVar[k].varType)>>2);
				aciTxSendPacket(ACIMT_SINGLEREQ,&temp_buffer[0],3+((aciListVar[k].varType)>>2));
//
			}
//			else
//				aciTxSendPacket(ACIMT_SENDVARTABLEENTRYINVALID,&selectedId,2);
		}
		break;
	}
}

void aciReceiveHandler(unsigned char rxByte)
{
	static unsigned char aciRxState=ARS_IDLE;
	static unsigned char aciRxMessageType;
	static unsigned short aciRxLength;
	static unsigned short aciRxCrc;
	static unsigned short aciRxReceivedCrc;


	switch (aciRxState)
	{
	case ARS_IDLE:
		if (rxByte=='!')
			aciRxState=ARS_STARTBYTE1;
		break;
	case ARS_STARTBYTE1:
		if (rxByte=='#')
			aciRxState=ARS_STARTBYTE2;
		else
			aciRxState=ARS_IDLE;

		break;
	case ARS_STARTBYTE2:
		if (rxByte=='!')
			aciRxState=ARS_MESSAGETYPE;
		else
			aciRxState=ARS_IDLE;
		break;
	case ARS_MESSAGETYPE:
		aciRxMessageType=rxByte;
		aciRxCrc=0xff;
		aciRxCrc=aciCrcUpdate(aciRxCrc,rxByte);
		aciRxState=ARS_LENGTH1;

		break;
	case ARS_LENGTH1:
		aciRxLength=rxByte;
		aciRxCrc=aciCrcUpdate(aciRxCrc,rxByte);
		aciRxState=ARS_LENGTH2;
		break;
	case ARS_LENGTH2:
		aciRxLength|=rxByte<<8;
		if (aciRxLength>ACI_RX_BUFFER_SIZE)
			aciRxState=ARS_IDLE;
		else
		{
			aciRxCrc=aciCrcUpdate(aciRxCrc,rxByte);
			aciRxDataCnt=0;
			if (aciRxLength)
				aciRxState=ARS_DATA;
			else
				aciRxState=ARS_CRC1;
		}
		break;
	case ARS_DATA:
		aciRxCrc=aciCrcUpdate(aciRxCrc,rxByte);
		aciRxDataBuffer[aciRxDataCnt++]=rxByte;
		if (aciRxDataCnt==aciRxLength)
			aciRxState=ARS_CRC1;
		break;
	case ARS_CRC1:
		aciRxReceivedCrc=rxByte;
		aciRxState=ARS_CRC2;
		break;
	case ARS_CRC2:
		aciRxReceivedCrc|=rxByte<<8;
		if (aciRxReceivedCrc==aciRxCrc)
		{
			aciRxHandleMessage(aciRxMessageType,aciRxLength);
		}
		aciRxState=ARS_IDLE;

		break;

	}

}
/*
 *
 * ACI TX Ringbuffer
 *
 */

void aciSetStartTxCallback(void (*aciStartTxCallback_func)(unsigned char))
{
	aciStartTxCallback=aciStartTxCallback_func;
}


void aciTxSendPacket(unsigned char aciMessageType, void * data, unsigned short cnt)
{
	unsigned char startstring[3] = { '!', '#', '!' };
	unsigned short crc = 0xFF;

	if (cnt + 10 >= aciTxRingBufferGetFreeSpace())
		return;

	//add header to ringbuffer
	aciTxRingBufferAddData(&startstring, 3);

	//add message type to ringbuffer
	aciTxRingBufferAddData(&aciMessageType, 1);
	crc=aciUpdateCrc16(crc,&aciMessageType,1);

	//add data size to ringbuffer
	aciTxRingBufferAddData(&cnt, 2);
	crc=aciUpdateCrc16(crc,&cnt,2);

	aciTxRingBufferAddData(data,cnt);
	crc=aciUpdateCrc16(crc,data,cnt);

	//add CRC to ringbuffer
	aciTxRingBufferAddData(&crc, 2);

}

void aciTxRingBufferReset(void)
{
	aciTxRingBufferWritePtr=0;
	aciTxRingBufferReadPtr=0;
}

unsigned short aciTxRingBufferGetFreeSpace(void)
{

	if (aciTxRingBufferWritePtr>=aciTxRingBufferReadPtr)
		return (ACI_TX_RINGBUFFER_SIZE-aciTxRingBufferWritePtr+aciTxRingBufferReadPtr-1);
	else
		return (aciTxRingBufferReadPtr-aciTxRingBufferWritePtr-1);


}

void aciTxRingBufferAddData(void * ptr, unsigned short size)
{
        unsigned char * ptrToChr=(unsigned char *)ptr;
        unsigned char triggerSend=0;

     if (!size)
    	 return;
	if (aciTxRingBufferGetFreeSpace()<size)
		return;

	if (aciTxRingBufferGetFreeSpace()==(ACI_TX_RINGBUFFER_SIZE-1))
		triggerSend=1;

	//check if data fits in without overflow of the write pointer
	if (ACI_TX_RINGBUFFER_SIZE-aciTxRingBufferWritePtr>size)
	{
		//it does. Copy everything at ones
		memcpy(&aciTxRingBuffer[aciTxRingBufferWritePtr],ptrToChr,size);
		aciTxRingBufferWritePtr+=size;
	}  else  {
		//it does not. We need two steps
		//copy to the end of the buffer
		memcpy(&aciTxRingBuffer[aciTxRingBufferWritePtr],ptrToChr,ACI_TX_RINGBUFFER_SIZE-aciTxRingBufferWritePtr);
                memcpy(&aciTxRingBuffer[0],ptrToChr+(ACI_TX_RINGBUFFER_SIZE-aciTxRingBufferWritePtr),size-(ACI_TX_RINGBUFFER_SIZE-aciTxRingBufferWritePtr));

		//increase and overflow correctly
		aciTxRingBufferWritePtr+=size;
		aciTxRingBufferWritePtr%=ACI_TX_RINGBUFFER_SIZE;

	}

	if ((triggerSend) && (aciStartTxCallback))
	{
		//get one byte from ringbuffer
		unsigned char byte=aciTxRingBufferGetNextByte();
		aciStartTxCallback(byte);
	}
}

unsigned char aciTxRingBufferByteAvailable(void)
{
	if (aciTxRingBufferGetFreeSpace()==ACI_TX_RINGBUFFER_SIZE-1)
		return 0;
	else
		return 1;
}

unsigned char aciTxRingBufferGetNextByte(void)
{

	unsigned char byte;

	if (!aciTxRingBufferByteAvailable())
		return 0;

	byte=aciTxRingBuffer[aciTxRingBufferReadPtr];
	aciTxRingBufferReadPtr++;
	aciTxRingBufferReadPtr%=ACI_TX_RINGBUFFER_SIZE;

	return byte;
}

/*
 *
 * ACI Helper functions
 *
 *
 */

unsigned short aciCrcUpdate (unsigned short crc, unsigned char data)
     {
         data ^= (crc & 0xff);
         data ^= data << 4;

         return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
                 ^ ((unsigned short )data << 3));
     }

unsigned short aciUpdateCrc16(unsigned short crc, void * data, unsigned short cnt)
{
	int i=0;
	unsigned short crcNew=crc;
	unsigned char * chrData=(unsigned char *)data;

	for (i=0;i<cnt;i++)
		crcNew=aciCrcUpdate(crcNew,chrData[i]);

	return crcNew;
}

void aciSetReadParafromFlashCallback(short (*aciReadParafromFlashCallback_func)(void))
{
	aciReadParafromFlashCallback = aciReadParafromFlashCallback_func;
}

void aciSetSaveParaCallback(void (*aciSaveParaCallback_func)(void))
{
	aciSaveParaCallback=aciSaveParaCallback_func;
}

void aciSetWriteParatoFlashCallback(short (*aciWriteParatoFlashCallback_func)(void))
{
	aciWriteParatoFlashCallback=aciWriteParatoFlashCallback_func;
}

