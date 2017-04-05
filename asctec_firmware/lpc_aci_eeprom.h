#ifndef ACI_EEPROM_H_
#define ACI_EEPROM_H_

// Defined Callbacks for LPC2k Processors for writing/reading from EEPROM

#include "asctecCommIntfOnboard.h"
#define _EEPROM_
#include "LPC2k_ee.h"					/* LPC2000 EEPROM definitions */
#undef _EEPROM_

#define COUNT_PAGES 1 // Max Count of Pages to save

extern struct ee_data content[COUNT_PAGES];

extern void lpc_aci_init(void); // Set Callbacks
extern short lpc_aci_ReadParafromFlash(void); // Read all Parameters and set them
extern void lpc_aci_SavePara(void); // Save parameter to Struct
extern short lpc_aci_WriteParatoFlash(void); // write struct to flash

#endif /* ACI_EEPROM_H_ */
