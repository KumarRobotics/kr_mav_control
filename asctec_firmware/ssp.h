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


#ifndef __SSP_H__
#define __SSP_H__

#include <stdint.h>

volatile unsigned int SSP_trans_cnt;

/* SPI read and write buffer size */
#define FIFOSIZE	8

/* SPI Status register */
#define SSPSR_TFE	1 << 0
#define SSPSR_TNF	1 << 1
#define SSPSR_RNE	1 << 2
#define SSPSR_RFF	1 << 3
#define SSPSR_BSY	1 << 4

/* SPI 1 CR0 register */
#define SSPCR0_DSS	1 << 0
#define SSPCR0_FRF	1 << 4
#define SSPCR0_CPOL	1 << 6
#define SSPCR0_CPHA	1 << 7
#define SSPCR0_SCR	1 << 8

/* SPI 1 CR1 register */
#define SSPCR1_LBM	1 << 0
#define SSPCR1_SSE	1 << 1
#define SSPCR1_MS	1 << 2
#define SSPCR1_SOD	1 << 3

/* SPI 1 Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM	1 << 0
#define SSPIMSC_RTIM	1 << 1
#define SSPIMSC_RXIM	1 << 2
#define SSPIMSC_TXIM	1 << 3

/* SPI 1 Interrupt Status register */
#define SSPRIS_RORRIS	1 << 0
#define SSPRIS_RTRIS	1 << 1
#define SSPRIS_RXRIS	1 << 2
#define SSPRIS_TXRIS	1 << 3

/* SPI 1 Masked Interrupt register */
#define SSPMIS_RORMIS	1 << 0
#define SSPMIS_RTMIS	1 << 1
#define SSPMIS_RXMIS	1 << 2
#define SSPMIS_TXMIS	1 << 3

/* SPI 1 Interrupt clear register */
#define SSPICR_RORIC	1 << 0
#define SSPICR_RTIC	1 << 1

extern void SSPHandler (void) __irq;
int LL_write(uint8_t *, uint16_t, uint8_t);
void LL_write_init(void);

extern uint8_t IMU_CalcData_updated;

#endif  /* __SSP_H__ */
/*****************************************************************************
**                            End Of File
******************************************************************************/

