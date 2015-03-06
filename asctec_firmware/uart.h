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

#ifndef __UART_H
#define __UART_H

#include <stdint.h>

extern void UARTInitialize(unsigned int);
extern void UART1Initialize(unsigned int baud);
extern void UARTWriteChar(uint8_t);
extern void UART1WriteChar(uint8_t);
extern uint8_t UARTReadChar(void);
extern uint8_t UART1ReadChar(void);
extern void __putchar(int);
extern void UART_send(uint8_t*, uint8_t);
extern void UART1_send(uint8_t*, uint8_t);
extern void UART_send_ringbuffer(void);
extern void UART1_send_ringbuffer(void);
extern int ringbuffer(uint8_t, uint8_t*, unsigned int);
extern int ringbuffer1(uint8_t, uint8_t*, unsigned int);
extern void uart0ISR(void);
extern void uart1ISR(void);
extern void UART_SendPacket(void*, uint8_t, uint8_t);

extern uint16_t crc16(void*, uint16_t);
extern uint16_t crc_update(uint16_t, uint8_t);

extern uint8_t trigger_transmission;
extern uint8_t transmission_running;

extern uint8_t Ctrl_Input_updated;
extern uint8_t PWM_Input_updated;

#define RBREAD 0
#define RBWRITE 1
#define RBFREE  2
#define RINGBUFFERSIZE	384

#endif //__UART_H

