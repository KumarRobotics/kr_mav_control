#ifndef SWI_H_
#define SWI_H_

extern unsigned long IntGetCPSR(void);

extern unsigned long IntDisable(void);
extern unsigned long IntEnable(void);
extern void IntRestore(unsigned long oldstate);

extern unsigned long FiqDisable(void);
extern unsigned long FiqEnable(void);
extern void FiqRestore(unsigned long oldstate);

#endif

