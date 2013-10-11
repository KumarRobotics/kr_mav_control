/*****************************************************************************
 *   i2c.h:  Header file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *   Modifications from Ascending Technologies GmbH
 *
******************************************************************************/
#ifndef __I2C_H 
#define __I2C_H

#define BUFSIZE			0x20
#define MAX_TIMEOUT		0x00FFFFFF

#define I2CMASTER		0x01
#define I2CSLAVE		0x02

/* For more info, read Philips's SE95 datasheet */
#define SE95_ADDR		0x9E
#define SE95_ID			0x05
#define SE95_CONFIG		0x01
#define SE95_TEMP		0x00
#define RD_BIT			0x01

#define GET_DEVICE_ID		0x01
#define GET_TEMPERATURE		0x02
#define SET_CONFIGURATION	0x03

#define I2C_IDLE		0
#define I2C_STARTED		1
#define I2C_RESTARTED		2
#define I2C_REPEATED_START	3
#define DATA_ACK		4
#define DATA_NACK		5

#define I2CONSET_I2EN		0x00000040  /* I2C Control Set Register */
#define I2CONSET_AA		0x00000004
#define I2CONSET_SI		0x00000008
#define I2CONSET_STO		0x00000010
#define I2CONSET_STA		0x00000020

#define I2CONCLR_AAC		0x00000004  /* I2C Control clear Register */
#define I2CONCLR_SIC		0x00000008
#define I2CONCLR_STAC		0x00000020
#define I2CONCLR_I2ENC		0x00000040

#define I2DAT_I2C		0x00000000  /* I2C Data Reg */
#define I2ADR_I2C		0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH		0x00000080  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL		0x00000080  /* I2C SCL Duty Cycle Low Reg */

extern void I2CInit( unsigned int I2cMode );
extern unsigned int I2CStart( void );
extern unsigned int I2CStop( void );
extern unsigned int I2CEngine( void );
extern void I2C0_send_motordata(void);
extern void I2C0MasterHandler(void);

#endif /* end __I2C_H */
/****************************************************************************
**                            End Of File
*****************************************************************************/
