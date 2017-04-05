/*

Copyright (c) 2011, Ascending Technologies GmbH
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


#ifndef UBLOX_H_
#define UBLOX_H_

#define UR_MAX_RAWDATA_LENGTH 384

extern void uBloxReceiveHandler(unsigned char recByte);
extern void uBloxReceiveEngine(void); //call with 200ms
extern unsigned char gpsLEDTrigger;

struct __attribute__((packed)) UBX_NAV_SOL
{
        unsigned int iTow;
        unsigned int fTow;
        short week;
        unsigned char gpsFix;
        unsigned char flags;
        int ecefX;
        int ecefY;
        int ecefZ;
        unsigned int pAcc;
        int ecefVX;
        int ecefVY;
        int ecefVZ;
        unsigned int sAcc;
        unsigned short pDOP;
        unsigned char reserved1;
        unsigned char numSV;
        unsigned int reserved2;
};

struct __attribute__((packed)) UBX_NAV_POSLLH
{
        unsigned int iTow;
        int lon;
        int lat;
        int height;
        int hMSL;
        int hAcc;
        int vAcc;
};

struct __attribute__((packed)) UBX_NAV_VELNED
{
        unsigned int iTow;
        int velN;
        int velE;
        int velD;
        unsigned int speed;
        unsigned int gSpeed;
        int heading;
        unsigned int sAcc;
        unsigned int cAcc;
};

struct __attribute__((packed)) UBX_NAV_STATUS
{
        unsigned int iTow;
        unsigned char gpsFix;
        unsigned char flags;
        unsigned char fixStat;
        unsigned char flags2;
        unsigned int ttff;
        unsigned int msss;
};

struct __attribute__((packed)) UBX_MON_VER_HEADER
{
        char swVersion[30];
        char hwVersion[10];
        char romVersion[30];
};

struct __attribute__((packed)) UBX_MON_SCHED
{
        unsigned char reserved[16];
        unsigned short stackUsed;
        unsigned short stackAv;
        unsigned short cpuLoad;
        unsigned char fullSlots;
        unsigned char partSlots;
};

struct __attribute__((packed)) UBX_MON_HW
{
        unsigned int pinSel;
        unsigned int pinBank;
        unsigned int pinDir;
        unsigned int pinVal;

        unsigned short noisePerMs;
        unsigned short agcCnt;
        unsigned char aStatus;
        unsigned char aPower;
        unsigned char flags;
        unsigned char reserved1;
        unsigned int usedMast;
        unsigned char vp[25];
        unsigned char jamInd;
        unsigned short reserved3;
        unsigned int pinIrq;
        unsigned int pullH;
        unsigned int pullL;
};

struct __attribute__((packed)) UBX_NAV_SVINFO_HEADER
{
        unsigned int iTow;
	unsigned char numCh;
	unsigned char globalFlags;
        unsigned short reserved2;
};

struct __attribute__((packed)) UBX_NAV_SVINFO_SAT
{
	unsigned char channel;
	unsigned char svId;
	unsigned char flags;
	unsigned char quality;
	unsigned char cNo;
	char elev; //in deg
	short azim; // in deg
	int prRes; //pseudo range residual
};

struct __attribute__((packed)) UBX_CFG_CFG
{
        unsigned int clearMask;
        unsigned int saveMask;
        unsigned int loadMask;
        unsigned char deviceMask;
};

struct __attribute__((packed)) SVINFO_HEADER
{
	unsigned char numCh;
	unsigned char globalFlags;
};

struct __attribute__((packed)) SVINFO
{
	unsigned char channel;
	unsigned char svId;
	unsigned char flags;
	unsigned char quality;
	unsigned char cNo;
	char elev; //in deg
	short azim; // in deg
	int prRes; //pseudo range residual
};

struct __attribute__((packed)) GPS_HW_STATUS
{
	 unsigned char antennaStatus;
	 unsigned char antennaPower;
	 unsigned short agcMonitor;
	 unsigned short noiseLevel;

	 unsigned short stackUsed;
	 unsigned short stackAv;
	 unsigned short cpuLoad;
	 unsigned char fullSlots;
	 unsigned char partSlots;
};

extern struct GPS_HW_STATUS gpsHardwareStatus;

#endif /* UBLOX_H_ */
