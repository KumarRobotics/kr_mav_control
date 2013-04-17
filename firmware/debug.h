/****************************************************************************
*
*   Copyright (c) 2006 Carrick Detweiler
*                      and Massachusetts Institute of Technology
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
*   $Id: debug.h 142 2007-01-05 17:10:44Z carrick $
****************************************************************************/


#if !defined( DEBUG_H )
#define DEBUG_H
#include "stdio.h"
//#include "console.h"

/**
 * The default debug level (if not specified in the file itself)
 **/
#ifndef DEBUG
#define DEBUG 4
#endif

/**
 * Where debug printfs should go to
 **/
//#define DEBUG_PRINTF printf
#define DEBUG_PRINTF(...) 

/**
 * If the debug output should be pretty (a little nicer to read?), but
 * this requires the allocation of some memory and the use of snprintf
 **/
//#define DEBUG_PRETTY_PRINT

/**
 * Prints file and line info.
 **/
#ifndef DEBUG_PRETTY_PRINT
#define DEBUG_PRINTLINE(pre,...)                       \
  {                                                    \
    DEBUG_PRINTF(pre );                                \
    DEBUG_PRINTF("%s:%d %s() ",                        \
                 __FILE__,__LINE__,__FUNCTION__);      \
    DEBUG_PRINTF( __VA_ARGS__ );                       \
  }
#else /* DEBUG_PRETTY_PRINT */

#define DEBUG_PRINTLINE(pre,...)                       \
  { char __debugTmp[45];                               \
    DEBUG_PRINTF(pre );                                \
    snprintf(__debugTmp,45,"%-18s %-4d %s()",     \
                 __FILE__,__LINE__,__FUNCTION__);      \
    DEBUG_PRINTF("%-45s |  ",__debugTmp);                   \
    DEBUG_PRINTF( __VA_ARGS__ );                       \
    DEBUG_PRINTG("\n");
  }
#endif

/**
 * Info printing
 **/
#define PRINTF(...) 
//#define PRINTF(...) 


/**
 * Just a macro to indicate that something still needs to be done
 **/
#define TBD(...)                                                 \
  DEBUG_PRINTLINE("TBD:    ",__VA_ARGS__);                       


#if DEBUG > 0
/**
 * Debug printing at level 1 debug.  Usage:
 *   DEBUG1(("value %d",val));
 **/
#define DEBUG1(...)                             \
  DEBUG_PRINTLINE("DEBUG1: ",__VA_ARGS__);

#if DEBUG > 1
/**
 * Debug printing at level 2 debug.  Usage:
 *   DEBUG2("value %d",val);
 **/
#define DEBUG2(...)                             \
  DEBUG_PRINTLINE("DEBUG2: ",__VA_ARGS__);

#if DEBUG > 2
/**
 * Debug printing at level 3 debug.  Usage:
 *   DEBUG3("value %d",val);
 **/
#define DEBUG3(...)                             \
  DEBUG_PRINTLINE("DEBUG3: ",__VA_ARGS__);

#if DEBUG > 3
/**
 * Debug printing at level 4 debug.  Usage:
 *   DEBUG4("value %d",val);
 **/
#define DEBUG4(...)                             \
  DEBUG_PRINTLINE("DEBUG4: ",__VA_ARGS__);

#else
#define DEBUG4(...)
#endif /* DEBUG > 3 */
#else
#define DEBUG4(...)
#define DEBUG3(...)
#endif /* DEBUG > 2 */
#else
#define DEBUG4(...)
#define DEBUG3(...)
#define DEBUG2(...)
#endif /* DEBUG > 1 */
#else
#define DEBUG4(...)
#define DEBUG3(...)
#define DEBUG2(...)
#define DEBUG1(...)
#endif /* DEBUG > 0 */


#endif /* DEBUG_H */
