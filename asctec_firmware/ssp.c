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

#include "LPC214x.h"			/* LPC21XX Peripheral Registers	*/
#include "type.h"
#include "irq.h"
#include "ssp.h"
#include "main.h"
#include "system.h"
#include "LL_HL_comm.h"
#include "sdk.h"

uint8_t SPIWRData[128];
uint8_t SPIRDData[128];
int CurrentTxIndex;
int CurrentRxIndex;
unsigned int SPIWR_num_bytes;

volatile unsigned int SSP_trans_cnt=0;

uint8_t data_sent_to_LL=1;

uint8_t SSP_receiption_complete=1;

uint8_t data_sent_to_HL=1;

void __irq SSPHandler (void)
{
    int regValue;
    uint16_t input_data;
//    uint8_t timeout=0;

    IENABLE;				/* handles nested interrupt */

    regValue = SSPMIS;
    if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
    {
		SSPICR = SSPICR_RORIC;		/* clear interrupt */
    }
    if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
    {
		SSPICR = SSPICR_RTIC;		/* clear interrupt */
    }

    if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
    {
    			/* receive until it's empty */
	while ( SSPSR & SSPSR_RNE )
	{
		input_data=SSPDR;

		SSP_rx_handler_HL(input_data&0xFF);
		SSP_rx_handler_HL(input_data>>8);

		//SSP_trans_cnt+=2;
	    /* Wait until the Busy bit is cleared */
	//    while ( (!(SSPSR & SSPSR_BSY) )&&(timeout++<50) );
	}				/* interrupt will be cleared when */
					/* data register is read or written */
    }

    if ( regValue & SSPMIS_TXMIS )	/* Tx at least half empty */
    {
	/* transmit until it's full */
	while ( (SSPSR & SSPSR_TNF) )
	{
	    if(CurrentTxIndex<SPIWR_num_bytes)
	    {
	    	SSPDR = SPIWRData[CurrentTxIndex]|(SPIWRData[CurrentTxIndex+1]<<8);
	    	CurrentTxIndex+=2;
	    }
	    else
	    {
	    	CurrentTxIndex=0;
	    	SPIWR_num_bytes=0;
	    	data_sent_to_LL=1;
			SSPDR=0;
	    }

	    /* Wait until the Busy bit is cleared */
	//    while ( !(SSPSR & SSPSR_BSY) );
	}				/* interrupt will be cleared when */
					/* data register is read or written */
    }

    IDISABLE;
    VICVectAddr = 0;		/* Acknowledge Interrupt */
}

void LL_write_init(void)
{
		SPIWRData[0]='>';
		SPIWRData[1]='*';
		SPIWRData[2]='>';
}

int LL_write(uint8_t *data, uint16_t cnt, uint8_t PD )	//write data to high-level processor
{
	unsigned int i;

	if(data_sent_to_LL)
	{
		SPIWRData[3]=PD;
		for(i=0; i<cnt; i++)
		{
			SPIWRData[i+4]=data[i];
		}
		SPIWRData[cnt+4]=0;
		SPIWR_num_bytes=cnt+5;
	}
	else if(SPIWR_num_bytes+cnt<127)
	{
		SPIWRData[SPIWR_num_bytes-1]='>';
		SPIWRData[0+SPIWR_num_bytes]='*';
		SPIWRData[1+SPIWR_num_bytes]='>';
		SPIWRData[2+SPIWR_num_bytes]=PD;
		for(i=SPIWR_num_bytes; i<cnt+SPIWR_num_bytes; i++)
		{
			SPIWRData[i+3]=data[i-SPIWR_num_bytes];
		}
		SPIWR_num_bytes+=cnt+5;
		SPIWRData[SPIWR_num_bytes-1]=0;
	}
	else return(0);
	data_sent_to_LL=0;

	return(1);
}


