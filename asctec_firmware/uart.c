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

#include "LPC214x.h"
#include "system.h"
#include "main.h"
#include "uart.h"
#include "irq.h"
#include "hardware.h"
#include "gpsmath.h"
#include "ssp.h"
#include "sdk.h"
#include "ublox.h"
#include "asctecCommIntfOnboard.h"

unsigned char packets;
unsigned char DataOutputsPerSecond;
unsigned int uart_cnt;

unsigned char data_requested=0;
extern int ZeroDepth;

unsigned short current_chksum;
unsigned char chksum_to_check=0;
unsigned char chksum_trigger=1;

unsigned char transmission_running=0;
unsigned char transmission1_running=0;
unsigned char trigger_transmission=0;

volatile unsigned char baudrate1_change=0;

unsigned char send_buffer[16];
unsigned char *tx_buff;
unsigned char UART_syncstate=0;
unsigned char UART1_syncstate=0;
unsigned int UART_rxcount=0;
unsigned char *UART_rxptr;
unsigned int UART1_rxcount=0;
unsigned char *UART1_rxptr;

unsigned char UART_CalibDoneFlag = 0;

static volatile unsigned char rb_busy=0;

unsigned char startstring[]={'>','*','>'};
unsigned char stopstring[]={'<','#','<'};


void uart1ISR(void) __irq
{
  unsigned char t;
  IENABLE;
  unsigned iir = U1IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
    {
      case 1:
		  // THRE interrupt
		 if (ringbuffer1(RBREAD, &t, 1))
		 {
		   transmission1_running=1;
		   UART1WriteChar(t);
		 }
		 else
		 {
		   transmission1_running=0;
		 }
        break;
      case 2:
    	// RX interrupt
	    uBloxReceiveHandler(U1RBR);
	    break;
      case 3:
        // RLS interrupt
        break;
      case 6:
        // CTI interrupt
        break;
   }
  IDISABLE;
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}


void uart0ISR(void) __irq
{
  unsigned char UART_rxdata;
#ifdef MATLAB
  unsigned char t;
#endif

  // Read IIR to clear interrupt and find out the cause
  IENABLE;
  unsigned iir = U0IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
    {
      case 1:
#ifdef MATLAB
    	  if (UART_Matlab_fifo(RBREAD, &t, 1))
    	 		 {
    	 		   transmission_running=1;
    	 		   UARTWriteChar(t);
    	 		 }
    	 		 else
    	 		 {
    	 		   transmission_running=0;
    	 		 }
#else
    	  if (aciTxRingBufferByteAvailable())
  			UARTWriteChar(aciTxRingBufferGetNextByte());
#endif
		break;

      case 2:
        // RDA interrupt - put your HL_serial_0 receive state machine here!
        UART_rxdata = U0RBR;
#ifdef MATLAB
        if (UART_syncstate==0)
        		{
        			if (UART_rxdata=='>') UART_syncstate++; else UART_syncstate=0;
        		}
        		else if (UART_syncstate==1)
        		{
        			if (UART_rxdata=='*') UART_syncstate++; else UART_syncstate=0;
        		}
        		else if (UART_syncstate==2)
        		{
        			if (UART_rxdata=='>') UART_syncstate++; else UART_syncstate=0;
        		}
        		else if (UART_syncstate==3)
        		{
        			if (UART_rxdata=='p') //data pending p=flight params
        			{
        				UART_syncstate=4;
        				UART_rxcount = sizeof(matlab_params_tmp);
        				UART_rxptr = (unsigned char*) &matlab_params_tmp;

        			}
        			else if (UART_rxdata=='c') //data pending c=uart ctrl
        			{
        				UART_syncstate = 5;
        				UART_rxcount = 24+2;
        				UART_rxptr = (unsigned char*) &matlab_uart_tmp;
        			}
        			else if (UART_rxdata=='s') //data pending s=save to eeprom
        			{
        				UART_syncstate=0;
        				triggerSaveMatlabParams=1;
        			}
        			else if (UART_rxdata=='h') // stop debug transmission
        			{
        				UART_syncstate=0;
        				xbee_send_flag=0;
        			}
                    else
                    	UART_syncstate=0;
                }
          		else if (UART_syncstate==4)
        		{
        			UART_rxcount--;
        			*UART_rxptr=UART_rxdata;
        			UART_rxptr++;
        			if (UART_rxcount==0)
                	{
                     	unsigned short crc_comp=0;
                     	UART_syncstate=0;
                     	crc_comp = crc16(&matlab_params_tmp, sizeof(matlab_params_tmp)-4);
                     	if (crc_comp==matlab_params_tmp.crc)
                     	{
                     		memcpy(&matlab_params, &matlab_params_tmp, sizeof(matlab_params));
                			//parameter_beep=ControllerCyclesPerSecond/10;
                     	}

                	}
        		}
          		else if (UART_syncstate==5)
        		{
        			UART_rxcount--;
        			*UART_rxptr=UART_rxdata;
        			UART_rxptr++;
        			if (UART_rxcount==0)
        			{
        				unsigned short crc_comp=0;
        				UART_syncstate=0;
        				crc_comp = crc16(&matlab_uart_tmp, 24);
        				if (crc_comp == matlab_uart_tmp.crc)
        				{
        					memcpy(&matlab_uart, &matlab_uart_tmp, sizeof(matlab_uart));
        					xbee_send_flag=1;
        				}


        			}
        		}
        		else UART_syncstate=0;
#else
        aciReceiveHandler(UART_rxdata);
        if (UART_syncstate==0)
		{
			if (UART_rxdata=='>') UART_syncstate++; else UART_syncstate=0;
		}
		else if (UART_syncstate==1)
		{
			if (UART_rxdata=='*') UART_syncstate++; else UART_syncstate=0;
		}
		else if (UART_syncstate==2)
		{
			if (UART_rxdata=='>') UART_syncstate++; else UART_syncstate=0;
		}
		else if (UART_syncstate==3)
		{
			if (UART_rxdata=='b')
			{
				UART_syncstate=4;
			}else
				UART_syncstate=0;

			//synchronized to start string => receive your data from here
        }
		else if (UART_syncstate==4)
		{
			if (UART_rxdata=='o')
			{
				UART_syncstate=5;
			}else if (UART_rxdata=='s')
			{
				//send sync
				UARTWriteChar('S');
				UART_syncstate=0;
			}
			else
				UART_syncstate=0;

			//synchronized to start string => receive your data from here
        }
		else if (UART_syncstate==5)
		{
			if (UART_rxdata=='o')
			{
				UART_syncstate=6;
			}else
				UART_syncstate=0;

			//synchronized to start string => receive your data from here
        }
		else if (UART_syncstate==6)
		{
			if (UART_rxdata=='t')
			{
				//start LPC bootloader
				UART_syncstate=0;
				enter_isp();
			}

			UART_syncstate=0;

			//synchronized to start string => receive your data from here
        }
		else UART_syncstate=0;
#endif

		break;
      case 3:
        // RLS interrupt
        break;
      case 6:
        // CTI interrupt
        break;
  }
  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
 }


void UARTInitialize(unsigned int baud)
{
  unsigned int divisor = peripheralClockFrequency() / (16 * baud);

  //UART0
  U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80; /* Disable DLAB */
  U0FCR = 1;


}

void UART1Initialize(unsigned int baud)
{
  unsigned int divisor = peripheralClockFrequency() / (16 * baud);
//UART1
  U1LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U1DLL = divisor & 0xFF;
  U1DLM = (divisor >> 8) & 0xFF;
  U1LCR &= ~0x80; /* Disable DLAB */
  U1FCR = 1;
}


//Write to UART0
void UARTWriteChar(unsigned char ch)
{
  while ((U0LSR & 0x20) == 0);
  U0THR = ch;
}
//Write to UART1
void UART1WriteChar(unsigned char ch)
{
  while ((U1LSR & 0x20) == 0);
  U1THR = ch;
}

unsigned char UARTReadChar(void)
{
  while ((U0LSR & 0x01) == 0);
  return U0RBR;
}

unsigned char UART1ReadChar(void)
{
  while ((U1LSR & 0x01) == 0);
  return U1RBR;
}

void __putchar(int ch)
{
  if (ch == '\n')
    UARTWriteChar('\r');
  UARTWriteChar(ch);
}

void UART_send(char *buffer, unsigned char length)
{
  unsigned char cnt=0;
  while (!(U0LSR & 0x20)); //wait until U0THR and U0TSR are both empty
  while(length--)
  {
    U0THR = buffer[cnt++];
    if(cnt>15)
    {
      while (!(U0LSR & 0x20)); //wait until U0THR is empty
    }
  }
}

void UART1_send(unsigned char *buffer, unsigned char length)
{
  unsigned char cnt=0;
  while(length--)
  {
    while (!(U1LSR & 0x20)); //wait until U1THR is empty
    U1THR = buffer[cnt++];
  }
}


void UART_send_ringbuffer(void)
{
  unsigned char t;
  if(!transmission_running)
  {
    if(ringbuffer(RBREAD, &t, 1))
    {
      transmission_running=1;
      UARTWriteChar(t);
    }
  }
}

void UART1_send_ringbuffer(void)
{
  unsigned char t;
  if(!transmission1_running)
  {
    if(ringbuffer1(RBREAD, &t, 1))
    {
      transmission1_running=1;
      UART1WriteChar(t);
    }
  }
}

void UART_SendPacket(void *data, unsigned short count, unsigned char packetdescriptor) //example to send data packets as on LL_serial_0
{
  unsigned short crc;
  int state;
      state=ringbuffer(RBWRITE, startstring, 3);
      state=ringbuffer(RBWRITE, (unsigned char *) &count, 2);
      state=ringbuffer(RBWRITE, &packetdescriptor, 1);
      state=ringbuffer(RBWRITE, data, count);
                crc=crc16(data,count);
      state=ringbuffer(RBWRITE, (unsigned char *) &crc, 2);
      state=ringbuffer(RBWRITE, stopstring, 3);
      UART_send_ringbuffer();
}

//example CRC16 function
unsigned short crc_update (unsigned short crc, unsigned char data)
     {
         data ^= (crc & 0xff);
         data ^= data << 4;

         return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
                 ^ ((unsigned short )data << 3));
     }

 unsigned short crc16(void* data, unsigned short cnt)
 {
   unsigned short crc=0xff;
   unsigned char * ptr=(unsigned char *) data;
   int i;

   for (i=0;i<cnt;i++)
     {
       crc=crc_update(crc,*ptr);
       ptr++;
     }
   return crc;
 }

// no longer a ringbuffer! - now it's a FIFO
int ringbuffer(unsigned char rw, unsigned char *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
    static volatile unsigned char buffer[RINGBUFFERSIZE];
//	static volatile unsigned int pfirst=0, plast=0;	//Pointers to first and last to read byte
	static volatile unsigned int read_pointer, write_pointer;
	static volatile unsigned int content=0;
	unsigned int p=0;
    unsigned int p2=0;

	if(rw==RBWRITE)
	{
		if(count<RINGBUFFERSIZE-content)	//enough space in buffer?
		{
			while(p<count)
			{
				buffer[write_pointer++]=data[p++];
			}
            content+=count;
            return(1);
		}
	}
	else if(rw==RBREAD)
	{
		if(content>=count)
		{
			while(p2<count)
			{
				data[p2++]=buffer[read_pointer++];
			}
            content-=count;
            if(!content) //buffer empty
            {
            	write_pointer=0;
            	read_pointer=0;
            }
			return(1);
		}
	}
        else if(rw==RBFREE)
        {
          if(content) return 0;
          else return(RINGBUFFERSIZE-11);
        }

	return(0);
}

int ringbuffer1(unsigned char rw, unsigned char *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
    static volatile unsigned char buffer[RINGBUFFERSIZE];
//	static volatile unsigned int pfirst=0, plast=0;	//Pointers to first and last to read byte
	static volatile unsigned int read_pointer, write_pointer;
	static volatile unsigned int content=0;
	unsigned int p=0;
    unsigned int p2=0;

	if(rw==RBWRITE)
	{
		if(count<RINGBUFFERSIZE-content)	//enough space in buffer?
		{
			while(p<count)
			{
				buffer[write_pointer++]=data[p++];
			}
            content+=count;
            return(1);
		}
	}
	else if(rw==RBREAD)
	{
		if(content>=count)
		{
			while(p2<count)
			{
				data[p2++]=buffer[read_pointer++];
			}
            content-=count;
            if(!content) //buffer empty
            {
            	write_pointer=0;
            	read_pointer=0;
            }
			return(1);
		}
	}
        else if(rw==RBFREE)
        {
          if(content) return 0;
          else return(RINGBUFFERSIZE-11);
        }

	return(0);
}

#ifdef MATLAB
void UART_Matlab_Initialize(unsigned int baud)
{
	unsigned int Divisor=0, MulVal=1, DivAddVal=0;

	// Line Control Register
	U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */

	if (baud == 3000000)
	{
		Divisor = 1;
		MulVal = 13;
		DivAddVal = 3;
	}
	else
	{
		Divisor = peripheralClockFrequency() / (16 * baud);
		MulVal = 1;
		DivAddVal = 0;
	}
	// Set Divisor
	U0DLL = Divisor & 0xFF;
	U0DLM = (Divisor >> 8) & 0xFF;
	// Set Fractional
	if (MulVal < 1)
		MulVal = 1;
	U0FDR = ((MulVal & 0xF) << 4) | (DivAddVal & 0xF);
	// Disable DLAB
	U0LCR &= ~0x80;
	// Enable FIFO
	U0FCR = 1;
}

void UART_Matlab_send_ringbuffer(void)
{
  unsigned char t;
  if(!transmission_running)
  {
    if(UART_Matlab_fifo(RBREAD, &t, 1))
    {
      transmission_running=1;
      UARTWriteChar(t);
    }
  }
}


void UART_Matlab_SendPacket(void *data, unsigned short count, unsigned char packetdescriptor)
{
  unsigned short crc;
  int state=0;
  crc=crc16(data,count);
  // Disable UART Interrupts
  U0IER = 0;
  state+=UART_Matlab_fifo(RBWRITE, startstring, 3);
  state+=UART_Matlab_fifo(RBWRITE, (unsigned char *) &count, 2);
  state+=UART_Matlab_fifo(RBWRITE, &packetdescriptor, 1);
  state+=UART_Matlab_fifo(RBWRITE, data, count);
  state+=UART_Matlab_fifo(RBWRITE, (unsigned char *) &crc, 2);
  state+=UART_Matlab_fifo(RBWRITE, stopstring, 3);
  UART_Matlab_send_ringbuffer();
  // Enable UART Interrupts
  U0IER = 3;
}


// no longer a ringbuffer! - now it's a FIFO
int UART_Matlab_fifo(unsigned char rw, unsigned char *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
	static volatile unsigned char buffer[MATLABFIFOSIZE];
	static volatile unsigned short i_read=0, i_write=0;
	unsigned int i=0;
	int return_val=0;

	if(rw == RBWRITE){
		while (i < count) {
			buffer[i_write++] = data[i++];
			i_write &= MATLABFIFOSIZE-1;
			if (i_write == i_read) {
				i_read++;
				i_read &= 128-1;
			}
		}
		return_val = 1;
	}
	else if(rw == RBREAD){
		while (i < count && i_write != i_read) {
			data[i++] = buffer[i_read++];
			i_read &= MATLABFIFOSIZE-1;
		}
		return_val = i;
	}
	return(return_val);
}
#endif
