/***********************************************************************/
/*                                                                     */
/*  SYSCALLS.C:  System Calls Remapping                                */
/*  most of this is from newlib-lpc and a Keil-demo                    */
/*                                                                     */
/*  these are "reentrant functions" as needed by                       */
/*  the WinARM-newlib-config, see newlib-manual                        */
/*  collected and modified by Martin Thomas                            */
/*  some more work has to be done on this		                       */
/***********************************************************************/

#include <stdlib.h>
#include <reent.h>
#include <sys/stat.h>
#include "uart.h"

// new code for _read_r provided by Alexey Shusharin - Thanks
_ssize_t _read_r(struct _reent *r, int file, void *ptr, size_t len)
{
  char c;
  int  i;
  unsigned char *p;

  p = (unsigned char*)ptr;

  for (i = 0; i < len; i++)
  {
    c = UARTReadChar();

    *p++ = c;
    UARTWriteChar(c);

    if (c == 0x0D && i <= (len - 2))
    {
      *p = 0x0A;
      UARTWriteChar(0x0A);
      return i + 2;
    }
  }
  return i;
}


#if 0
// old version - no longer in use
_ssize_t _read_r(
    struct _reent *r, 
    int file, 
    void *ptr, 
    size_t len)
{
	char c;
	int  i;
	unsigned char *p;
	
	p = (unsigned char*)ptr;
	
	for (i = 0; i < len; i++) {
		// c = uart0Getch();
		c = UARTReadChar();
		if (c == 0x0D) {
			*p='\0';
			break;
		}
		*p++ = c;
		UARTWriteChar(c);
	}
	return len - i;
}
#endif

_ssize_t _write_r (
    struct _reent *r, 
    int file, 
    const void *ptr, 
    size_t len)
{
	int i;
	const unsigned char *p;
	
	p = (const unsigned char*) ptr;
	
	for (i = 0; i < len; i++) {
		if (*p == '\n' ) UARTWriteChar('\r');
		UARTWriteChar(*p++);
	}
	
	return len;
}

int _close_r(
    struct _reent *r, 
    int file)
{
	return 0;
}

_off_t _lseek_r(
    struct _reent *r, 
    int file, 
    _off_t ptr, 
    int dir)
{
	return (_off_t)0;	/*  Always indicate we are at file beginning.	*/
}


int _fstat_r(
    struct _reent *r, 
    int file, 
    struct stat *st)
{
	/*  Always set as character device.				*/
	st->st_mode = S_IFCHR;	
		/* assigned to strong type with implicit 	*/
		/* signed/unsigned conversion.  Required by 	*/
		/* newlib.					*/

	return 0;
}

int isatty(int file); /* avoid warning */

int isatty(int file)
{
	return 1;
}

#if 0
static void _exit (int n) {
label:  goto label; /* endless loop */
}
#endif 

/* "malloc clue function" */

	/**** Locally used variables. ****/
extern char _end[];              /*  end is set in the linker command 	*/
				/* file and is the end of statically 	*/
				/* allocated data (thus start of heap).	*/

static char *heap_ptr;		/* Points to current end of the heap.	*/

/************************** _sbrk_r *************************************/
/*  Support function.  Adjusts end of heap to provide more memory to	*/
/* memory allocator. Simple and dumb with no sanity checks.		*/
/*  struct _reent *r	-- re-entrancy structure, used by newlib to 	*/
/*			support multiple threads of operation.		*/
/*  ptrdiff_t nbytes	-- number of bytes to add.			*/
/*  Returns pointer to start of new heap area.				*/
/*  Note:  This implementation is not thread safe (despite taking a	*/
/* _reent structure as a parameter).  					*/
/*  Since _s_r is not used in the current implementation, the following	*/
/* messages must be suppressed.						*/

void * _sbrk_r(
    struct _reent *_s_r, 
    ptrdiff_t nbytes)
{
	char  *base;		/*  errno should be set to  ENOMEM on error	*/

	if (!heap_ptr) {	/*  Initialize if first time through.		*/
		heap_ptr = _end;
	}
	base = heap_ptr;	/*  Point to end of heap.			*/
	heap_ptr += nbytes;	/*  Increase heap.				*/
	
	return base;		/*  Return pointer to start of new heap area.	*/
}
