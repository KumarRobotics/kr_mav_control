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
#include "sdk.h"

uint8_t transmission_running=0;
uint8_t transmission1_running=0;
uint8_t trigger_transmission=0;

static const uint8_t startstring[]={0x55,0x55};

uint8_t Ctrl_Input_updated = 0;

void uart1ISR(void) __irq
{
  uint8_t t;

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
      //uBloxReceiveHandler(U1RBR);
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
  uint8_t t;
  uint8_t UART_rxdata;

  static uint8_t sync_state = 0;
  static unsigned int rx_count = 0;
  static uint8_t *rx_ptr;
  static uint8_t received_type;
  static uint8_t received_length;
  static uint16_t expected_crc, received_crc;

  // Read IIR to clear interrupt and find out the cause
  IENABLE;
  const uint32_t iir = U0IIR;
  // Handle UART interrupt
  switch ((iir >> 1) & 0x7)
  {
    case 1:
      // THRE interrupt
      if(!(IOPIN0 & (1<<CTS_RADIO)))
      {
        trigger_transmission=0;
        if (ringbuffer(RBREAD, &t, 1))
        {
          transmission_running=1;
          UARTWriteChar(t);
        }
        else
        {
          transmission_running=0;
        }
      }
      else
      {
        trigger_transmission=1;
      }
      break;

    case 2:
      // RDA interrupt - put your HL_serial_0 receive state machine here!
      UART_rxdata = U0RBR;

      switch(sync_state)
      {
        case 0:
          if (UART_rxdata == startstring[0]) sync_state++;
          else sync_state = 0;
          break;
        case 1:
          if (UART_rxdata == startstring[1]) sync_state++;
          else sync_state = 0;
          break;
        case 2:
          received_length = UART_rxdata;
          expected_crc = crc16(&received_length, 1);
          sync_state++;
          break;
        case 3:
          received_type = UART_rxdata;
          expected_crc = crc_update(expected_crc, UART_rxdata);
          rx_count = 0;
          if( (received_type == TYPE_SO3_CMD) && (received_length == sizeof(struct SO3_CMD_INPUT)) )
          {
            rx_ptr = (uint8_t*)&SO3_cmd_input_tmp;
            sync_state++;
          }
          else
            sync_state = 0;
          break;
        case 4:
          *rx_ptr = UART_rxdata;
          rx_ptr++;
          rx_count++;
          expected_crc = crc_update(expected_crc, UART_rxdata);

          if(rx_count == received_length) sync_state++;
          break;
        case 5:
          received_crc = UART_rxdata;
          sync_state++;
          break;
        case 6:
          received_crc = 256*UART_rxdata + received_crc;
          if(received_crc == expected_crc)
          {
            // Received full packet
            Ctrl_Input_updated = 1;
          }
          sync_state = 0;
          break;
        default:
          sync_state = 0;
          break;
      }
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
void UARTWriteChar(uint8_t ch)
{
  while ((U0LSR & 0x20) == 0);
  U0THR = ch;
}
//Write to UART1
void UART1WriteChar(uint8_t ch)
{
  while ((U1LSR & 0x20) == 0);
  U1THR = ch;
}

uint8_t UARTReadChar(void)
{
  while ((U0LSR & 0x01) == 0);
  return U0RBR;
}

uint8_t UART1ReadChar(void)
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

#if 0
void UART_send(uint8_t *buffer, uint8_t length)
{
  uint8_t cnt=0;
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
#else
void UART_send(uint8_t *buffer, uint8_t length)
{
  uint8_t cnt=0;
  while(length--)
  {
    while (!(U0LSR & 0x20)); //wait until U0THR is empty
    U0THR = buffer[cnt++];
  }
}
#endif

void UART1_send(uint8_t *buffer, uint8_t length)
{
  uint8_t cnt=0;
  while(length--)
  {
    while (!(U1LSR & 0x20)); //wait until U1THR is empty
    U1THR = buffer[cnt++];
  }
}


void UART_send_ringbuffer(void)
{
  uint8_t t;
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
  uint8_t t;
  if(!transmission1_running)
  {
    if(ringbuffer1(RBREAD, &t, 1))
    {
      transmission1_running=1;
      UART1WriteChar(t);
    }
  }
}

void UART_SendPacket(void *data, uint8_t count, uint8_t packetdescriptor) //example to send data packets as on LL_serial_0
{
  uint16_t crc;
  int state;
  state = ringbuffer(RBWRITE, (uint8_t *) startstring, 2);
  state = ringbuffer(RBWRITE, (uint8_t *) &count, 1);
  crc = crc16(&count, 1);
  state = ringbuffer(RBWRITE, &packetdescriptor, 1);
  crc = crc_update(crc, packetdescriptor);
  state = ringbuffer(RBWRITE, data, count);
  for(int i = 0; i < count; i++)
    crc = crc_update(crc, ((uint8_t*)data)[i]);
  state = ringbuffer(RBWRITE, (uint8_t *) &crc, 2);
  UART_send_ringbuffer();
}

// CRC-16-ITU-T function
uint16_t crc_update (uint16_t crc, uint8_t data)
{
  uint16_t x = ((crc >> 8) ^ data);
  x ^= x >> 4;

  crc = (crc << 8) ^ (x << 12) ^ (x <<5) ^ x;

  return crc;
}

uint16_t crc16(void* data, uint16_t cnt)
{
  uint16_t crc = 0xffff;
  uint8_t * ptr=(uint8_t *) data;
  int i;

  for (i=0;i<cnt;i++)
  {
    crc=crc_update(crc,*ptr);
    ptr++;
  }
  return crc;
}

// no longer a ringbuffer! - now it's a FIFO
int ringbuffer(uint8_t rw, uint8_t *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
  static volatile uint8_t buffer[RINGBUFFERSIZE];
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

int ringbuffer1(uint8_t rw, uint8_t *data, unsigned int count)	//returns 1 when write/read was successful, 0 elsewise
{
  static volatile uint8_t buffer[RINGBUFFERSIZE];
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
