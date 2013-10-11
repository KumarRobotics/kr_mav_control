/****************************************************************************************/
/*                                                                            			*/
/*  LPC2k_ee.C:  basic LPC213x EEPROM routines (rev 1.1, Jan 27th, 2006.)				*/
/*                                                                            			*/
/* Functions provided here:																*/
/*   	                                                    		                    */
/* ee_erase(command_ee, result_ee[]):	erases all EEPROM								*/
/* ee_write(command_ee, result_ee[]):	writes record of ee_data (defined in LPC2k_ee.h)*/
/* ee_read(command_ee, result_ee[]) :	reads the last record added into EEPROM			*/
/* ee_readn(command_ee, result_ee[]):	reads the n-th record in EEPROM					*/
/* ee_count(command_ee, result_ee[]):	counts records of ee_data type in EEPROM		*/
/*			   	                                                                        */
/****************************************************************************************/

#include "LPC214x.h"
#define _EEPROM_
#include "LPC2k_ee.h"					/* LPC2000 EEPROM definitions */
#undef _EEPROM_
#define IAP_LOCATION 			0x7ffffff1

//const uint8_t eeprom[EE_SIZE] _at_ EE_ADDR_L;
void ee_erase(unsigned int ,unsigned int[]);	//function erases EEPROM
void ee_write(unsigned int ,unsigned int[]);	//function adds a record in EEPROM
void ee_read (unsigned int ,unsigned int[]);	//function reads the latest valid record in EEPROM
void ee_count(unsigned int ,unsigned int[]);	//function counts records in EEPROM
void ee_readn(unsigned int ,unsigned int[]);	//function reads n-th record in EEPROM
int ee_locate(void);							   	//function locates the next available location

typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP iap_entry;

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_erase(unsigned int command_ee,unsigned int result_ee[])		*/
/*                                                                     	*/
/* type: void                                                          	*/
/*                                                                     	*/
/* parameters: 															*/
/* 	command_ee   - Not used.  	                              			*/
/*  result_ee[0] - Returns a response to the last IAP command used.		*/
/*                 0 - EEPROM successfully erased.						*/
/*                 For all other response values, see microcontroller 	*/
/*				   User Manual, IAP Commands and Status Codes Summary.	*/
/*  result_ee[1] - Not used.  	                              			*/
/*                                                                     	*/
/* version: 1.1 (01/27/2006)                                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  EE_SEC_L 	 - microcontroller's Flash sector where EEPROM begins	*/
/*  EE_SEC_H 	 - microcontroller's Flash sector where EEPROM ends		*/
/*  EE_CCLK		 - microcontroller's system clock (cclk)                */
/*                                                                     	*/
/* description:															*/
/*  This function erases LPC2000 on-chip Flash sectors selected to act 	*/
/*  as an EEPROM. All Flash sectors between EE_SEC_L abd EE_SEC_H		*/
/*  (including these sectors) will be erased using the In Application	*/
/*  Programming (IAP) routines (see User Manual for more details). 		*/
/*  Also, this function disables all interrupts while erasing the       */
/*  EEPROM. If this is not needed, three lines of the ee_erase          */
/*  subroutine can simply be commented-out without affecting the        */
/*  routine performance at all.                                         */
/*                                                                     	*/
/* revision history:                                                   	*/
/* - Rev. 1.1 adds interrupt disable feature.							*/
/*                                                                     	*/
/************************************************************************/
void ee_erase(unsigned int command_ee,unsigned int result_ee[]){
	unsigned int command_iap[5];
	unsigned int result_iap[3];
	unsigned long int enabled_interrupts;

	enabled_interrupts = VICIntEnable;  //disable all interrupts
	VICIntEnClr        = enabled_interrupts;

	command_iap[0]=50;					//prepare sectors from EE_SEC_L to EE_SEC_H for erase
	command_iap[1]=EE_SEC_L;
	command_iap[2]=EE_SEC_H;
	iap_entry=(IAP) IAP_LOCATION;
	iap_entry(command_iap,result_iap);

	command_iap[0]=52;					//erase sectors from EE_SEC_L to EE_SEC_H
	command_iap[1]=EE_SEC_L;
	command_iap[2]=EE_SEC_H;
	command_iap[3]=EE_CCLK;
	iap_entry=(IAP) IAP_LOCATION;
	iap_entry(command_iap,result_iap);

	command_iap[0]=53;					//blankcheck sectors from EE_SEC_L to EE_SEC_H
	command_iap[1]=EE_SEC_L;
	command_iap[2]=EE_SEC_H;
	iap_entry=(IAP) IAP_LOCATION;
	iap_entry(command_iap,result_iap);

	VICIntEnable = enabled_interrupts;  //restore interrupt enable register

	result_ee[0]=result_iap[0];
	return;
}

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_write(unsigned int command_ee,unsigned int result_ee[])		*/
/*                                                                     	*/
/* type: void                                                          	*/
/*                                                                     	*/
/* parameters: 															*/
/* 	command_ee   - An address of a content of ee_data type that has		*/
/*                 to be programmed into EEPROM.                       	*/
/*  result_ee[0] - Returns a response to the last IAP command used.		*/
/*                 0 - data successfully programmed in EEPROM.			*/
/*               501 - no space in EEPROM to program data.             	*/
/*                 For all other response values, see microcontroller 	*/
/*				   User Manual, IAP Commands and Status Codes Summary.	*/
/*  result_ee[1] - Not used.  	                              			*/
/*                                                                     	*/
/* version: 1.1 (01/27/2006)                                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  EE_BUFFER_SIZE 	   - IAP buffer size; must be 256 or 512 			*/
/*  NO_SPACE_IN_EEPROM - EEPROM is full and no data can be programmed	*/
/*  EE_BUFFER_MASK	   - parameter used for interfacing with IAP		*/
/*  EE_REC_SIZE   	   - ee_data structure size in bytes        		*/
/*  EE_SEC_L 	 	   - micro's Flash sector where EEPROM begins		*/
/*  EE_SEC_H 	 	   - micro's Flash sector where EEPROM ends			*/
/*  EE_CCLK		 	   - micro's system clock (cclk)                	*/
/*                                                                     	*/
/* description:															*/
/*  This function writes a single structure of ee_data type into the	*/
/*  EEPROM using an In Application	Programming (IAP) routines (see 	*/
/*  User Manual for more details). command_ee contains an address of	*/
/*  this structure. EEPROM is scanned for the last (if any) record 		*/
/*  identifier (EE_REC_ID), and a new record is added next to it.      	*/
/*  Also, this function disables all interrupts while erasing the       */
/*  EEPROM. If this is not needed, three lines of the ee_write          */
/*  subroutine can simply be commented-out without affecting the        */
/*  routine performance at all.                                         */
/*                                                                     	*/
/* revision history:                                                   	*/
/* - Rev. 1.1 fixes a bug related to verifying a content written into	*/
/*   the EEPROM. 1.0 was reporting missmatch even when there were no	*/
/*   problems at all.													*/
/*   Rev. 1.1 adds interrupt disable feature.							*/
/*                                                                     	*/
/************************************************************************/
void ee_write(unsigned int command_ee,unsigned int result_ee[]){
	int location;
	unsigned int *source, *destination, i;
	uint8_t ee_buffer[EE_BUFFER_SIZE];
	unsigned int command_iap[5], result_iap[3];
	unsigned long int enabled_interrupts;

	location = ee_locate();
	if (location == -1){
		result_ee[0]=NO_SPACE_IN_EEPROM;
	}
	else{
		for (i=0;i<EE_BUFFER_SIZE;i++) ee_buffer[i]=0xFF;

		destination = (unsigned int *) ((&ee_buffer[0])+((unsigned int)location & EE_BUFFER_MASK));
		source = (unsigned int *) command_ee;
		for(i=0;i!=EE_REC_SIZE/4;i++) *(destination+i) = *(source+i);

		enabled_interrupts = VICIntEnable;  //disable all interrupts
		VICIntEnClr        = enabled_interrupts;

		command_iap[0]=50;					//prepare sectors from EE_SEC_L to EE_SEC_H for erase
		command_iap[1]=EE_SEC_L;
		command_iap[2]=EE_SEC_H;
		iap_entry=(IAP) IAP_LOCATION;
		iap_entry(command_iap,result_iap);

		command_iap[0]=51;					//copy RAM to flash/eeprom
		command_iap[1]=(unsigned int) (location & EE_START_MASK);
		command_iap[2]=(unsigned int) (&ee_buffer[0]);
		command_iap[3]=EE_BUFFER_SIZE;
		command_iap[4]=EE_CCLK;
		iap_entry=(IAP) IAP_LOCATION;
		iap_entry(command_iap,result_iap);

		command_iap[0]=56;					//compare RAM and flash/eeprom
		command_iap[1]=(unsigned int) source;
		command_iap[2]=(unsigned int) location;
		command_iap[3]=EE_REC_SIZE;
		iap_entry=(IAP) IAP_LOCATION;
		iap_entry(command_iap,result_iap);

		VICIntEnable = enabled_interrupts;  //restore interrupt enable register

		result_ee[0]=result_iap[0];
	}
	return;
}

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_read(unsigned int command_ee,unsigned int result_ee[])		*/
/*                                                                     	*/
/* type: void                                                          	*/
/*                                                                     	*/
/* parameters: 															*/
/* 	command_ee   - Not used.											*/
/*  result_ee[0] - Returns a response.									*/
/*                 0 - data successfully found in EEPROM.				*/
/*               500 - no data/records available in EEPROM.				*/
/*  result_ee[1] - an address of the last record of ee_data type		*/
/*				   in EEPROM.  	                              			*/
/*                                                                     	*/
/* version: 1.1 (01/27/2006)                                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  NO_RECORDS_AVAILABLE - EEPROM is empty/no records identifiable		*/
/*						   with a record identifier (EE_REC_ID) found	*/
/*  EE_ADR_L 	 	   - micro's Flash address from where EEPROM begins	*/
/*  EE_REC_SIZE 	   - size (in bytes) of a ee_data structure        	*/
/*                                                                     	*/
/* description:															*/
/*  This function scans an EEPROM content looking for the last record 	*/
/*  that can be identified with a record identifier (EE_REC_ID). When 	*/
/*  such data is found, its address is passed as result_ee[1].			*/
/*                                                                     	*/
/* revision history:                                                   	*/
/* - Rev. 1.0 had problems with accessing the last record in a fully	*/
/*   occupied EEPROM. Rev. 1.1 fixes this.								*/
/*                                                                     	*/
/************************************************************************/
void ee_read(unsigned int command_ee,unsigned int result_ee[]){
	int location;

	location = ee_locate();
	if (location == EE_ADDR_L){
		result_ee[0]=NO_RECORDS_AVAILABLE;
	}
	else{
		result_ee[0]=0;
		if (location == -1)
			result_ee[1]=(unsigned int)(EE_ADDR_H+1 - EE_REC_SIZE);
		else
			result_ee[1]=(unsigned int)(location - EE_REC_SIZE);
	}
	return;
}

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_readn(unsigned int command_ee,unsigned int result_ee[])		*/
/*                                                                     	*/
/* type: void                                                          	*/
/*                                                                     	*/
/* parameters: 															*/
/* 	command_ee   - An index of a record in EEPROM that should be read.	*/
/*  result_ee[0] - Returns a response.									*/
/*                 0 - data successfully found in EEPROM.				*/
/*               502 - requested index of record in EEPROM is out of 	*/
/*                     EEPROM's memory.                                	*/
/*  result_ee[1] - an address of the specified record of ee_data type	*/
/*				   in EEPROM.  	                              			*/
/*                                                                     	*/
/* version: 1.0 (initial release 05/13/2005)                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  INDEX_OUT_OF_RANGE - index of a record in EEPROM specified by 		*/
/*						 command_ee is out of EEPROM's range			*/
/*  EE_ADR_L 	 	   - micro's Flash address from where EEPROM begins	*/
/*  EE_ADR_H 	 	   - micro's Flash address where EEPROM ends		*/
/*  EE_REC_SIZE 	   - size (in bytes) of a ee_data structure        	*/
/*                                                                     	*/
/* description:															*/
/*  This function returns in result_ee[1] an address of an EEPROM 		*/
/*  record index specified in command_ee. Index can not be less than 0.	*/
/*                                                                     	*/
/************************************************************************/
void ee_readn(unsigned int command_ee,unsigned int result_ee[]){
	if(command_ee>((EE_ADDR_H+1-EE_ADDR_L)/EE_REC_SIZE)){
		result_ee[0]=INDEX_OUT_OF_RANGE;}
	else{
		result_ee[0]=0;
		result_ee[1]=(unsigned int)(EE_ADDR_L+EE_REC_SIZE*command_ee);
	}
	return;
}

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_count(unsigned int command_ee,unsigned int result_ee[])		*/
/*                                                                     	*/
/* type: void                                                          	*/
/*                                                                     	*/
/* parameters: 															*/
/* 	command_ee   - Not used.											*/
/*  result_ee[0] - Returns a response. Always 0.						*/
/*  result_ee[1] - number of records of ee_data type in EEPROM.			*/
/*                                                                     	*/
/* version: 1.1 (01/27/2006)                                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  EE_ADR_L 	 	   - micro's Flash address from where EEPROM begins	*/
/*  EE_REC_SIZE 	   - size (in bytes) of a ee_data structure        	*/
/*                                                                     	*/
/* description:															*/
/*  This function returns number of records of ee_data type in EEPROM.	*/
/*                                                                     	*/
/* revision history:                                                   	*/
/* - Initial release (1.0) was not supplying the right feedback in case */
/*   of counting records in a full EEPROM. Rev. 1.1 fixes this.         */
/*                                                                     	*/
/************************************************************************/
void ee_count(unsigned int command_ee,unsigned int result_ee[]){
	int location;
	result_ee[0]=0;
	location = ee_locate();
	if (location == -1) location = EE_ADDR_H+1;
	result_ee[1]=(unsigned int)((location-EE_ADDR_L)/EE_REC_SIZE);
	return;
}

/************************************************************************/
/*                                                                    	*/
/* function: 															*/
/*  void ee_locate()													*/
/*                                                                     	*/
/* type: int                                                          	*/
/*                                                                     	*/
/* parameters: none														*/
/*                                                                     	*/
/* version: 1.1 (01/27/2006)                                           	*/
/*                                                                     	*/
/* constants defined in LPC2k_ee.h used in this function:              	*/
/*  EE_ADR_L 	 	   - micro's Flash address from where EEPROM begins	*/
/*  EE_ADR_H 	 	   - micro's Flash address where EEPROM ends		*/
/*  EE_REC_ID 	 	   - a record indicator used to identify valid data	*/
/*  EE_REC_SIZE 	   - size (in bytes) of a ee_data structure        	*/
/*                                                                     	*/
/* description:															*/
/*  This function returns an address as of which new record can be 		*/
/*	added into Flash/EEPROM. In case of EEPROM being full, function     */
/*  returns -1. Searching is based on divide by two method that         */
/*  provides the fastest processing time.                               */
/*                                                                     	*/
/* revision history:                                                   	*/
/* - Rev. 1.1 fixes a bug related to identifying an unused byte of 		*/
/*   EEPROM in an EEPROM with size not equal to EE_REC_SIZE * 2^k (k>=0)*/
/*                                                                     	*/
/************************************************************************/
int ee_locate(void){
	unsigned int addr_l, addr_m, addr_r, size, slice_limit;
	addr_l = EE_ADDR_L;
	if ((*((uint8_t *)addr_l))==0xFF) return(addr_l);
	addr_r = EE_ADDR_H+1;
	if ((*((uint8_t *)(addr_r-EE_REC_SIZE)))==EE_REC_ID) return(-1);
	size = addr_r - addr_l;
	slice_limit = EE_REC_SIZE - 1;
	while(size != EE_REC_SIZE){
		addr_m = (addr_r+addr_l)/2;
		if ((addr_m & slice_limit)!=0x00000000){
			if ((*((uint8_t *)(addr_r - EE_REC_SIZE)))==0xFF)
				addr_r = addr_r - EE_REC_SIZE;
			else
				addr_l = addr_l + EE_REC_SIZE;
			addr_m = (addr_r+addr_l)/2;
			size = size - EE_REC_SIZE;
		}
		if ((*((uint8_t *)addr_m))==0xFF)
			addr_r = addr_m;
		else
			addr_l = addr_m;
		size = size/2;
	}
	return(addr_r);
}
