/*****************************************************************************
 *   i2c.h:  Header file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *
 ******************************************************************************/
#ifndef __I2C1_H
#define __I2C1_H

#define BUFSIZE					0x20

#define I2C_IDLE		0
#define I2C_STARTED		1
#define I2C_RESTARTED		2
#define I2C_REPEATED_START	3
#define DATA_ACK		4
#define DATA_NACK		5
#define DATA_READ_DONE  6
#define DATA_WRITE_DONE 7
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
#define I21SCLH_SCLH		0x00000200  /* I2C SCL Duty Cycle High Reg */
#define I21SCLL_SCLL		0x00000200  /* I2C SCL Duty Cycle Low Reg */

extern void I2C1Init(void);
extern void I2C1MasterHandler(void);
extern unsigned int I2C1Engine(void);
extern unsigned char I2C1State(void);
extern void fireFlySetLed(unsigned char r, unsigned char g, unsigned char b);
extern void fireFlyLedHandler(void);
extern char I2C1_setRGBLed(unsigned char r, unsigned char g, unsigned char b);

#define I2C_ERROR_NONE 0x00
#define I2C_ERROR_NACKAFTERSTART 0x01
#define I2C_ERROR_NACKAFTERWRITE 0x02
#define I2C_ERROR_NACKAFTERREAD 0x03
#define I2C_ERROR_BUFSIZETOSMALL 0x04
#define I2C_ERROR_BUSBUSY 0x05
#define I2C_ERROR_NODATA 0x06

#endif /* end __I2C_H */
/****************************************************************************
 **                            End Of File
 *****************************************************************************/
