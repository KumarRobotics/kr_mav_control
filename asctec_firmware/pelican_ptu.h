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

#ifndef PELICAN_PTU_H_
#define PELICAN_PTU_H_


#define HUMMINGBIRD_ROLL_SERVO	//generate roll servo output and use HL serial 0 TX for PWM 1
#define HUMMINGBIRD_ROLL_SERVO_ON_SSEL0	//SSEL0 is used for Roll servo, serial 0 TX stays TX pin!
#define CAMMOUNT_XCONFIG // turn roll/pitch commands for camera compensation by 45°
#define PELICAN_PTU //roll/nick servo offset and scales for standard Pelican camera mount, pitch dir -1, roll dir 1
#define CAM_FACING_FRONT_RIGHT	//define if camera is facing to the front right, for front left leave undefined

//Servo angle offsets in degrees*1000 => change this value if camera mount is not leveled
#define CAMERA_OFFSET_HUMMINGBIRD_PITCH	0
#define CAMERA_OFFSET_HUMMINGBIRD_ROLL	0

#define HUMMINGBIRD_SERVO_DIRECTION_PITCH 1 //1: servo mounted on right hand side of camera, -1: left
#define HUMMINGBIRD_SERVO_DIRECTION_ROLL  -1

void PTU_init(void);
void PTU_update(void);
void PTU_update_middle_positions_by_stick(void);
void SERVO_pitch_move(int);
void SERVO_roll_move(int);

extern int PTU_cam_angle_roll_offset;
extern int PTU_cam_angle_pitch_offset;
extern unsigned char PTU_enable_plain_ch7_to_servo; // =1->channel 7 is mapped plain to 1-2ms servo output

extern unsigned char PTU_cam_option_4_version;

struct CAMERA_PTU {	//Pan Tilt Unit Data

	int servo_roll_offset;
	int servo_pitch_offset;

	int servo_roll_scale;
	int servo_pitch_scale;

	int servo_pitch_min;
	int servo_pitch_max;

	int servo_roll_min;
	int servo_roll_max;
};

extern struct CAMERA_PTU CAMERA_ptu;

struct CAMERA_COMMANDS {

    unsigned short status;     //0x01 => camera power on; 0x00 => camera power off
    short chksum;  //status + desired_angle_pitch + desired_angle_roll;
	int desired_angle_pitch;	//desired angles in 1/1000th degree
	int desired_angle_roll;
};

extern struct CAMERA_COMMANDS CAMERA_Commands;

#endif /*PELICAN_PTU_H_*/

