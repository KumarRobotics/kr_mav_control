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

#define a	6378137.0								// earth semimajor axis in meters
#define f 	0.0033528106647474807198455286185206	// reciprocal flattening
#define e2 	2*f-f*f									// eccentricity squared
#define MEAN_EARTH_RADIUS 	6378137.0
#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR	0.017453292519943295769236907684886		//PI/180
#define PI 3.1415926535897932384626433832795


struct GPS_DATA
{	
//latitude/longitude in deg * 10^7
	int latitude;
	int longitude;
//GPS height in mm 
	int height;
//speed in x (E/W) and y(N/S) in mm/s	
	int speed_x;
	int speed_y;
//GPS heading in deg * 1000
	int heading; 
	
//accuracy estimates in mm and mm/s
	unsigned int horizontal_accuracy;
	unsigned int vertical_accuracy;
	unsigned int speed_accuracy;

//number of satellite vehicles used in NAV solution
	unsigned int numSV;

// GPS status information; Bit7...Bit3: 0 Bit 2: longitude direction Bit1: latitude direction Bit 0: GPS lock
	int status; 		
};
extern struct GPS_DATA GPS_Data;

struct GPS_TIME 
{
	unsigned int time_of_week;	//[ms]
	unsigned short week;		//[1..52]
};
extern struct GPS_TIME GPS_Time;


//trigger's new gps data transmission
extern unsigned int gpsDataOkTrigger;
extern void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon);	//X: East, Y: North in m; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement

