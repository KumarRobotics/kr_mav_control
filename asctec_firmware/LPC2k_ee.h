#ifndef LPC2K_EE_H_
#define LPC2K_EE_H_

/************************************************************************/
/*                                                                    	*/
/*			LPC2k_ee.H:  Header file enabling EEPROM support			*/
/* 		for Philips LPC2000 microcontroller's on-chip Flash memory		*/
/*       			(revision 1.0, May 13th, 2005.)						*/
/*                                                                     	*/
/*			This file is to be used with LPC2k_ee.c	file 			  	*/
/*                                                                     	*/
/* IMPORTANT: on-chip Flash memory sector(s) intended to be used as 	*/
/* an EEPROM will be unavailable for regular code storage! The smallest	*/
/* amount of Flash memory that can be used as an EEPROM is a single 	*/
/* Flash sector (regardless of the Flash sector actual size). 			*/
/*                                                                     	*/
/* If size of desired EEPROM requires several Flash sectors, these		*/
/* sectors must be a consecutive ones.									*/
/*                                                                     	*/
/************************************************************************/

#define EE_SEC_L                14                              //Flash sector where EEPROM begins (see UM for details)
#define EE_SEC_H                14                              //Flash sector where EEPROM ends (see UM for details)
#define EE_ADDR_L               0x00038000              //Must match the EE_SEC_L Flash sector start address
#define EE_ADDR_H               0x0003FFFF              //Must match the EE_SEC_H Flash sector end address

#define EE_CCLK			60000			//system clock cclk expressed in kHz (5*12 MHz)

/************************************************************************/
/*                                                                    	*/
/* ee_data structure can be defined differently from this example.		*/
/* The only requirement is to have _id field as it is defined here		*/
/* since EE_REC_ID character is used to identify a record's presence	*/
/* in the EEPROM memory.												*/
/*                                                                     	*/
/* ====================================================================	*/
/*                                                                     	*/
/* IMPORTANT ARM memory access considerations:							*/
/*                                                                     	*/
/* char		: byte alligned. Can be accessed at any location in memory.	*/
/*                                                                     	*/
/* short int: occupies 2 consecutive bytes. It can be read/write 		*/
/*			  accessed only when half-word alligned. Therefore, it is	*/
/* 			  located at addresses ending with 0x0, 0x2, 0x4, 0x6, 0x8,	*/
/*			  0xA, 0xC or 0xE.											*/
/*                                                                     	*/
/* int		: occupies 4 consecutive bytes.	It can be read/write 		*/
/*			  accessed only when half-word alligned. Therefore, it is	*/
/* 			  located at addresses ending with 0x0, 0x4, 0x8 or 0xC.	*/
/*                                                                     	*/
/* ====================================================================	*/
/*                                                                     	*/
/* Due to the LPC2000 Flash memory characteristics, an ee_data 			*/
/* structure size (EE_REC_SIZE) is limited to the following set:		*/
/*                                                                     	*/
/* LPC2101/2/3, LPC2131/2/4/6/8, LPC2141/2/4/6/8: 0x10, 0x20, 0x40,     */ 
/*                                                0x80 or 0x100         */
/*                                                                     	*/
/* LPC2104/5/6, LPC2112/4/9, LPC2124/9, LPC2192/4: 0x10, 0x20, 0x40,    */
/*                                                 0x80, 0x100 or 0x200 */
/*                                                                     	*/
/* ====================================================================	*/
/*                                                                     	*/
/* example1:                                                          	*/
/*                                                                     	*/
/* struct ee_data{					//structure starts as word alligned	*/
/*	unsigned char	_id;			//1 byte  - no allignement restr.	*/
/*                                  //  	    3 BYTE GAP!!!!         	*/		
/*	unsigned int	_rec_count;		//4 bytes - must be word alligned!	*/
/*	unsigned char	_cs;			//1 byte  - no allignement restr.	*/
/*};								// next structure will start as		*/
/*                                  // word alligned...                 */
/* Structure in example 1 occupies 12 bytes of memory					*/
/*                                                                     	*/
/* --------------------------------------------------------------------	*/
/*                               
                                      	*/
/* example2:                                                          	*/
/*                                                                     	*/
/* struct ee_data{					//structure starts as word alligned	*/
/*	unsigned char	_id;			//1 byte  - no allignement restr.	*/
/*	unsigned char	_cs;			//1 byte  - no allignement restr.	*/
/*                                  //  	    2 BYTE GAP!!!!         	*/		
/*	unsigned int	_rec_count;		//4 bytes - must be word alligned!	*/
/*};								// next structure will start as		*/
/*                                  // word alligned...                 */
/* Structure in example 2 occupies 8 bytes of memory					*/
/*                                                                     	*/
/************************************************************************/

struct ee_data{
	unsigned char data[254]; // 254 Bytes of  data
	unsigned char data_count; // count of data written
	unsigned char next_side; // active low!
};										// 256 Bytes total = one complete site

/************************************************************************/
/*																		*/
/* 	Disclaimer: all observations presented in example1, example 2 and 	*/
/*	ee_data structure defined here are based on Keil's ARM compiler. 	*/
/*	If another compiler is used, memory usage would have to be 		 	*/
/*	re-examined and verified.										 	*/
/*																		*/
/************************************************************************/


#define EE_REC_SIZE		0x100		//see restrictions from above

/********************************************************************/
/*																	*/
/* 						Valid combinations for 						*/
/* EE_REC_SIZE, EE_BUFFER_SIZE, EE_BUFFER_MASK and EE_START_MASK	*/
/*																	*/
/* EE_BUFFER_SIZE ! EE_START_MASK ! EE_REC_SIZE ! EE_BUFFER_MASK	*/
/* ----------------------------------------------------------------	*/
/*    	256			0xFFFFFF00			0x010		  	0xF0		*/
/*		256			0xFFFFFF00			0x020		 	0xE0		*/
/*    	256			0xFFFFFF00			0x040		  	0xC0		*/
/*		256			0xFFFFFF00			0x080		 	0x80		*/
/*		256			0xFFFFFF00			0x100			0x00		*/
/* ---------------------------------------------------------------- */
/*    	512			0xFFFFFE00			0x010		  	0x1F0		*/
/*		512			0xFFFFFE00			0x020		 	0x1E0		*/
/*    	512			0xFFFFFE00			0x040		  	0x1C0		*/
/*		512			0xFFFFFE00			0x080			0x180	 	*/
/*		512			0xFFFFFE00			0x100		 	0x100		*/
/*		512			0xFFFFFE00			0x200		 	0x000		*/
/********************************************************************/
/*	For LPC2101/2/3, LPC213x and LPC214x EE_BUFFER_SIZE is 256.		*/
/*	For all other LPC2000 devices EE_BUFFER_SIZE is always 512.		*/
/********************************************************************/
#define EE_BUFFER_SIZE	256
#define EE_START_MASK	0xFFFFFF00
#define EE_BUFFER_MASK	0x00000000

/********************************************************************/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!															  !!*/
/*!!															  !!*/
/*!! 				DO NOT MODIFY THE FOLLOWING CODE!!!			  !!*/
/*!!				===================================			  !!*/
/*!!															  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/********************************************************************/

#define EE_REC_ID 				0xAA
#define EE_SIZE					(EE_ADDR_H+1-EE_ADDR_L)
#define NO_RECORDS_AVAILABLE	500
#define NO_SPACE_IN_EEPROM		501
#define INDEX_OUT_OF_RANGE		502

#ifndef _EEPROM_
	extern const unsigned char eeprom[];
	extern void ee_erase(unsigned int , unsigned int []);	//function erases EEPROM
	extern void ee_write(unsigned int , unsigned int []);	//function adds a record in EEPROM
	extern void ee_read (unsigned int , unsigned int []);	//function reads the latest valid record in EEPROM
	extern void ee_readn(unsigned int , unsigned int []);	//function reads n-th record in EEPROM
	extern void ee_count(unsigned int , unsigned int []);	//function counts records in EEPROM
#endif

	extern void enter_isp(void);

#endif /* LPC2K_EE_H_ */
