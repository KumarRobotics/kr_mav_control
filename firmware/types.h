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
*   $Id: types.h 142 2007-01-05 17:10:44Z carrick $
****************************************************************************/


#if !defined( TYPES_H )
#define TYPES_H

/**
 * Different types that are needed.  It is assumed that the type is
 * signed unless they are eu types in which case they should be
 * unsigned.  Ideally (perhaps required?) that they have they same bit
 * length as indicated (eg uint8_t is an unsigned 8-bit type).
 **/
#define eint8 signed char
#define euint8 uint8_t
#define eint16 signed short
#define euint16 uint16_t
#define eint32 signed int
#define euint32 unsigned int
#define echar char
#define euchar uint8_t


#endif /* TYPES_H */
