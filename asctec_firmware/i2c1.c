/*****************************************************************************
 *   i2c.c:  I2C C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/
#include "LPC214x.h"                        /* LPC21xx definitions */
#include "type.h"
#include "irq.h"
#include "i2c1.h"
#include "main.h"
#include "LL_HL_comm.h"
#include "gpsmath.h"
#include "system.h"
#ifndef FALCON

DWORD I2C1MasterState = I2C_IDLE;

BYTE I2C1MasterBuffer[BUFSIZE];
DWORD I2C1Count = 0;
DWORD I2C1WriteLength;

DWORD WrIndex1 = 0;

unsigned char lastI2c1Error=I2C_ERROR_NONE;

/*
From device to device, the I2C communication protocol may vary,
in the example below, the protocol uses repeated start to read data from or
write to the device:
For master read: the sequence is: STA,Addr(W),offset,RE-STA,Addr(w),data...STO
for master write: the sequence is: STA,Addr(W),length,RE-STA,Addr(r),data...STO
Thus, in state 8, the address is always WRITE. in state 10, the address could
be READ or WRITE depending on the I2CCmd.
*/

/*****************************************************************************
** Function name:		I2C0MasterHandler
**
** Descriptions:		I2C0 interrupt handler, deal with master mode
**				only.
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/

void I2C1MasterHandler (void) __irq
{
    BYTE StatValue;

    /* this handler deals with master read and master write only */
    StatValue = I21STAT;

    IENABLE;
    switch ( StatValue )
    {
	case 0x08:			/* A Start condition is issued. */
	I21DAT = I2C1MasterBuffer[0];
	I21CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
	I2C1MasterState = I2C_STARTED;
	break;


	case 0x18:			/* Regardless, it's a ACK */
	if ( I2C1MasterState == I2C_STARTED )
	{
	    I21DAT = I2C1MasterBuffer[1];
	    WrIndex1++;
	    I2C1MasterState = DATA_ACK;
	}
	I21CONCLR = I2CONCLR_SIC;
	break;

	case 0x28:	/* Data byte has been transmitted, regardless ACK or NACK */
	if ( WrIndex1 < I2C1WriteLength )
	{
	    I21DAT = I2C1MasterBuffer[1+WrIndex1]; /* this should be the last one */
	    WrIndex1++;
	    I2C1MasterState = DATA_ACK;

	}
	else
	{
		I2C1MasterState = DATA_WRITE_DONE;
		  	//I2C Stop
		I21CONSET = I2CONSET_STO;      /* Set Stop flag */
		I21CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
	}
	I21CONCLR = I2CONCLR_SIC;
	lastI2c1Error=I2C_ERROR_NONE;

	break;

	case 0x30: //data has been transmited but NACK was received
		I2C1MasterState = DATA_NACK;
		  	//I2C Stop
		I21CONSET = I2CONSET_STO;      /* Set Stop flag */
		I21CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
		lastI2c1Error=I2C_ERROR_NACKAFTERWRITE;
	break;

	case 0x20: //NACK received, receiver not found
				I21CONCLR = I2CONCLR_SIC;
				I2C1MasterState = DATA_NACK;
				//I2C Stop
				I21CONSET = I2CONSET_STO;      /* Set Stop flag */
				I21CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
				lastI2c1Error=I2C_ERROR_NACKAFTERSTART;
	break;



	default:
	I21CONCLR = I2CONCLR_SIC;
	break;
    }

    IDISABLE;
    VICVectAddr = 0;		/* Acknowledge Interrupt */

}

void fireFlyLedHandler(void) 	//called with 100Hz
{
	unsigned char r,g,b;
	unsigned short errorFlags;
	static unsigned short cnt=0;
	static unsigned char mfsCnt=0;
	static unsigned char mincCnt=0;
	static unsigned char ceCnt=0;

	errorFlags=0;


	if (LL_1khz_attitude_data.flightMode&FM_CALIBRATION_ERROR_GYROS)
	{
		errorFlags|=0x02;
	}

	if (LL_1khz_attitude_data.flightMode&FM_CALIBRATION_ERROR_ACC)
	{
		errorFlags|=0x04;
	}

	if ((LL_1khz_attitude_data.flightMode&FM_COMPASS_FAILURE) && (ceCnt<5))
	{
		errorFlags|=0x08;
	}

	if ((LL_1khz_attitude_data.flightMode&FM_MAG_INCLINATION_ERROR) && (mincCnt<5) && ((LL_1khz_attitude_data.flightMode&FM_COMPASS_FAILURE)==0))
	{
		errorFlags|=0x10;
	}

	if ((LL_1khz_attitude_data.flightMode&FM_MAG_FIELD_STRENGTH_ERROR) && (mfsCnt<5) && ((LL_1khz_attitude_data.flightMode&FM_COMPASS_FAILURE)==0))
	{
		errorFlags|=0x20;
	}

	cnt++;
	if (cnt==200)
		cnt=0;
	if(!SYSTEM_initialized)
		{
			errorFlags|=0x01;
			r=0;
			g=0;
			b=255;
		}
	else if (((GPS_Data.status&0xFF)==3) && (HL_Status.battery_voltage_1>11000))
    {
    	r=0;
    	g=255;
    	b=0;
    }else if (HL_Status.battery_voltage_1<11000)
    {
    	unsigned int dim=11000-HL_Status.battery_voltage_1;

    	if (dim>1000)
    		dim=1000;


    	if ((GPS_Data.status&0xFF)!=3)
    	{
        	dim/=6;
        	dim+=10;
        	if (dim>165)
        		dim=165;
    		r=255;
    		g=165-dim;
    		b=0;
    	}else
    	{
        	dim/=4;
        	dim+=5;
        	if (dim>255)
        		dim=255;
    		r=dim;
    		g=255-dim;
    		b=0;
    	}
    	if(HL_Status.battery_voltage_1<10300) //blink LED @ very low voltage
    	{
    		if((cnt%20)<10) {r=0;g=0;b=0;};
    	}
    }else if ((GPS_Data.status&0xFF)!=3)
    {
    	r=255;
    	g=165;
    	b=0;
    }

	if ((cnt<=100) && (errorFlags))
	{
		r=g=b=0;
	}

	if ((cnt>100) && (cnt<200) && (errorFlags)) //always signal errors for 1 second.
	{
		//overwrite rgb with error signaling
		if (errorFlags & 0x02) //gyros
		{
			if (cnt < 125) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 150) {
				r = 255;
				g = 0;
				b = 0;

			} else if (cnt < 175) {
				r = 0;
				g = 0;
				b = 0;
			} else {
				r = 255;
				g = 0;
				b = 0;
			}
		} else if (errorFlags & 0x04) {
			//blink 3x
			if (cnt < 125) {
				r = 255;
				g = 0;
				b = 0;
			}
			if (cnt < 140) {
				r = 0;
				g = 0;
				b = 0;

			} else if (cnt < 175) {
				r = 255;
				g = 0;
				b = 0;
			} else if (cnt < 190) {
				r = 0;
				g = 0;
				b = 0;
			} else {
				r = 255;
				g = 0;
				b = 0;
			}
		} else if (errorFlags & 0x08) {
			if ((ceCnt < 5) && (cnt == 195))
				ceCnt++;
			//blink 5x
			if (cnt < 105) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 115) {
				r = 255;
				g = 0;
				b = 0;
			} else if (cnt < 125) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 135) {
				r = 255;
				g = 0;
				b = 0;
			} else if (cnt < 145) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 155) {
				r = 255;
				g = 0;
				b = 0;
			} else if (cnt < 165) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 175) {
				r = 255;
				g = 0;
				b = 0;
			} else if (cnt < 185) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 195) {
				r = 255;
				g = 0;
				b = 0;
			} else {
				r = 0;
				g = 0;
				b = 0;
			}
		} else if (errorFlags & 0x10) {
			if ((mincCnt < 5) && (cnt == 195))
				mincCnt++;
			//blink 4x
			if (cnt < 105) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 115) {
				r = 0;
				g = 0;
				b = 255;
			} else if (cnt < 125) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 135) {
				r = 0;
				g = 0;
				b = 255;
			} else if (cnt < 145) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 155) {
				r = 0;
				g = 0;
				b = 255;
			} else if (cnt < 165) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 175) {
				r = 0;
				g = 0;
				b = 255;
			} else {
				r = 0;
				g = 0;
				b = 0;
			}
		} else if (errorFlags & 0x20) {
			if ((mfsCnt < 5) && (cnt == 195))
				mfsCnt++;
			//blink 3x !
			if (cnt < 105) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 115) {
				r = 0;
				g = 0;
				b = 255;
			} else if (cnt < 125) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 135) {
				r = 0;
				g = 0;
				b = 255;
			} else if (cnt < 145) {
				r = 0;
				g = 0;
				b = 0;
			} else if (cnt < 155) {
				r = 0;
				g = 0;
				b = 255;
			} else {
				r = 0;
				g = 0;
				b = 0;
			}
		}
	}
	I2C1_setRGBLed(r, g, b);
}


char I2C1_setRGBLed(unsigned char r,unsigned char g,unsigned char b)
{
	if ((I2C1MasterState == I2C_STARTED) || (I2C1MasterState == DATA_ACK))
			return I2C_ERROR_BUSBUSY;
	WrIndex1=0;
	I2C1WriteLength = 5;
	I2C1MasterBuffer[0] = 0x30;	//address
    I2C1MasterBuffer[1] = r;
    I2C1MasterBuffer[2] = g;
    I2C1MasterBuffer[3] = b;
    I2C1MasterBuffer[4] = 255-(unsigned char)(r+g+b);

    I2C1MasterState = I2C_STARTED;
    I2C1Engine();

    return 0;
}



void I2C1Init(void)
{
    /*--- Clear flags ---*/
    I21CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

    /*--- Reset registers ---*/
    I21SCLL   = I21SCLL_SCLL;
    I21SCLH   = I21SCLH_SCLH;

    install_irq( I2C1_INT, (void *) I2C1MasterHandler );
    I21CONSET = I2CONSET_I2EN;
}

unsigned int I2C1Engine( void )
{
	I21CONSET = I2CONSET_STO;      /* Set Stop flag */
	I21CONCLR = I2CONCLR_SIC;  /* Clear SI flag */
	I2C1MasterState = I2C_IDLE;
    WrIndex1 = 0;
    I21CONSET = I2CONSET_STA;	/* Set Start flag */
    return ( TRUE );
}

unsigned char I2C1State(void)
{
	return I2C1MasterState;
}
#endif
