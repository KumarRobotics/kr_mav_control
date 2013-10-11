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

#include "gpsmath.h"
#include <math.h>

#define a	6378137.0								// earth semimajor axis in meters
#define f 	0.0033528106647474807198455286185206	// reciprocal flattening
#define e2 	2*f-f*f									// eccentricity squared
#define MEAN_EARTH_RADIUS 	6378137.0
#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR	0.017453292519943295769236907684886		//PI/180
#define PI 3.1415926535897932384626433832795

struct GPS_DATA GPS_Data;
struct GPS_DATA gps_data_temp;

unsigned int gpsDataOkTrigger=0;

void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon)	//X: East, Y: North in m; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement
{
        *lat=lat0+Y/MEAN_EARTH_DIAMETER*360./PI;
        *lon=lon0+X/MEAN_EARTH_DIAMETER*360./PI/cos(lat0*UMR);
}

