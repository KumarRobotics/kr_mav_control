#ifndef PELICAN_PTU_H_
#define PELICAN_PTU_H_


#define HUMMINGBIRD_ROLL_SERVO	//generate roll servo output and use HL serial 0 TX for PWM 1
#define HUMMINGBIRD_ROLL_SERVO_ON_SSEL0	//SSEL0 is used for Roll servo, serial 0 TX stays TX pin!
#define CAMMOUNT_XCONFIG // turn roll/pitch commands for camera compensation by 45°
#define PELICAN_PTU //roll/nick servo offset and scales for standard Pelican camera mount, pitch dir -1, roll dir 1
//#define CAM_FACING_FRONT_RIGHT	//define if camera is facing to the front right, for front left leave undefined

#define CAMERA_OFFSET_HUMMINGBIRD_PITCH	0
#define CAMERA_OFFSET_HUMMINGBIRD_ROLL	0
#define HUMMINGBIRD_SERVO_DIRECTION_PITCH -1 //1: servo mounted on right hand side of camera, -1: left
#define HUMMINGBIRD_SERVO_DIRECTION_ROLL  1

void PTU_init(void);
void PTU_update(int);
void SERVO_pitch_move(int);
void SERVO_roll_move(int);

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
