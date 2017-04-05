/*

Copyright (c) 2013, Ascending Technologies GmbH
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

#include "jetiTelemetry.h"
#include <string.h>

struct JETI_VALUE jetiValues[15];

unsigned char jetiName[10];

unsigned char jetiDisplayText[32];

unsigned char jetiAlarm=0;
unsigned char jetiAlarmType=0;
unsigned char jetiTriggerTextSync=0;

unsigned char jetiKeyChanged=0;
unsigned char jetiKey=0;


void jetiSetKeyChanged(unsigned char key)
{
	jetiKey=key;
	jetiKeyChanged=1;
}


unsigned char jetiCheckForKeyChange(void)
{
	if (jetiKeyChanged)
	{
		jetiKeyChanged=0;
		return jetiKey;
	}else
		return 0;
}
unsigned char jetiSetAlarm(unsigned char alarm, unsigned alarmType)
{
	if (((alarm<'A')||(alarm>'Z')) && (alarm))
		return JETI_ERROR_ALARM_RANGE;
	if (alarmType>1)
		return JETI_ERROR_ALARM_TYPE;
	jetiAlarm=alarm;
	jetiAlarmType=alarmType;

	return JETI_NO_ERROR;

}

unsigned char jetiSetDeviceName(char * name)
{
	unsigned char nameLength=0;
	int i;
	unsigned char error=JETI_NO_ERROR;

	for (i=0;i<11;i++)
		if (name[i]==0)
			break;
	if (i==10)
		error=JETI_ERROR_STRING_NAME;

	nameLength=i;

	memcpy(&jetiName[0],name,nameLength);

	return error;
}

unsigned char jetiActivateValue(unsigned char id)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;
	jetiValues[id].active=id+1;

	return JETI_NO_ERROR;
}

unsigned char jetiDeactivateValue(unsigned char id)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;
	jetiValues[id].active=0;

	return JETI_NO_ERROR;
}

unsigned char jetiSetDecimalPoint(unsigned char id, unsigned char decimalPoint)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;
	if (decimalPoint>3)
		return JETI_ERROR_DECPOINT_RANGE;
	jetiValues[id].decPoint=decimalPoint;
	return JETI_NO_ERROR;
}

unsigned char jetiSetValue30B(unsigned char id, int value)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;

	if ((value>=(1<<30)) || (value<=-(1<<30)))
		return JETI_ERROR_VALUE_RANGE;

	jetiValues[id].value=value;
	if (jetiValues[id].varType!=JETI_VART_30B)
	{
		jetiValues[id].varType=JETI_VART_30B;
	}

	return JETI_NO_ERROR;

}

unsigned char jetiSetValue22B(unsigned char id, int value)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;
	if ((value>=(1<<23)) || (value<=-(1<<23)))
		return JETI_ERROR_VALUE_RANGE;

	jetiValues[id].value=value;
	if (jetiValues[id].varType!=JETI_VART_22B)
	{
		jetiValues[id].varType=JETI_VART_22B;
	}

	return JETI_NO_ERROR;
}

unsigned char jetiSetValue6B(unsigned char id, int value)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;

	if ((value>=(1<<7)) || (value<=-(1<<7)))
		return JETI_ERROR_VALUE_RANGE;

	jetiValues[id].value=value;
	if (jetiValues[id].varType!=JETI_VART_6B)
	{
		jetiValues[id].varType=JETI_VART_6B;
	}

	return JETI_NO_ERROR;
}

unsigned char jetiSetValue14B(unsigned char id, int value)
{
	if (id>14)
		return JETI_ERROR_ID_RANGE;

	if ((value>=(1<<15)) || (value<=-(1<<15)))
		return JETI_ERROR_VALUE_RANGE;

	jetiValues[id].value=value;
	if (jetiValues[id].varType!=JETI_VART_14B)
	{
		jetiValues[id].varType=JETI_VART_14B;
	}

	return JETI_NO_ERROR;

}

unsigned char jetiSetValueTime(unsigned char id, unsigned char hours, unsigned char minutes, unsigned char seconds)
{
	unsigned int value;
	if (id>14)
		return JETI_ERROR_ID_RANGE;
	if (seconds>59)
		return JETI_ERROR_SECOND;
	if (minutes>59)
		return JETI_ERROR_MINUTE;
	if (hours>23)
		return JETI_ERROR_HOUR;

	value=seconds;
	value|=minutes<<8;
	value|=hours<<16;

	jetiValues[id].value=value;
	jetiSetDecimalPoint(id,0); //time
	jetiValues[id].varType=JETI_VART_DATETIME;

	return JETI_NO_ERROR;
}

unsigned char jetiSetValueDate(unsigned char id, unsigned char day, unsigned char month, unsigned short year)
{
	unsigned int value;
	if (id>14)
		return JETI_ERROR_ID_RANGE;

	if (year>2000)
		year-=2000;
	if (year>100)
		return JETI_ERROR_YEAR;
	if ((month<1)||(month>12))
		return JETI_ERROR_MONTH;
	if ((day<1)||(day>31))
		return JETI_ERROR_DAY;

	value=year;
	value|=month<<8;
	value|=day<<16;

	jetiValues[id].value=value;
	jetiSetDecimalPoint(id,1); //time
	jetiValues[id].varType=JETI_VART_DATETIME;

	return JETI_NO_ERROR;
}

unsigned char jetiSetTextDisplay(char * text)
{
	unsigned char textLength=0;
	unsigned char error=JETI_NO_ERROR;
	unsigned char textChanged=0;
	int i;

	for (i=0;i<33;i++)
	{
		if (text[i]!=jetiDisplayText[i])
			textChanged=1;
		if (text[i]==0)
			break;
	}
	if (i==32)
		error=JETI_ERROR_STRING_TEXT;

	textLength=i;
	memcpy(&jetiDisplayText[0],text,textLength);
	if (textLength<32)
		for (i=textLength;i<32;i++)
			jetiDisplayText[i]=0;

	if (textChanged)
		jetiTriggerTextSync=1;
	return error;
}

unsigned char jetiInitValue(unsigned char id, char * description, char * unit)
{
	unsigned char descLength=0;
	unsigned char unitLength=0;
	unsigned char error=JETI_NO_ERROR;

	int i;

	if (id>14)
		return JETI_ERROR_ID_RANGE;

	for (i=0;i<11;i++)
		if (description[i]==0)
			break;
	if (i==10)
		error=JETI_ERROR_STRING_DESC;
	descLength=i;

	for (i=0;i<6;i++)
			if (unit[i]==0)
				break;
	if (i==5)
			error=JETI_ERROR_STRING_UNIT;

	unitLength=i;

	jetiValues[id].active=id+1;
	memcpy(&jetiValues[id].name[0],description,descLength);
	memcpy(&jetiValues[id].unit[0],unit,unitLength);
	jetiSetValue6B(id,0);
	jetiSetDecimalPoint(id,0);

	return error;

}


