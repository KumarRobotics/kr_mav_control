/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#ifndef KQUADCOM_H
#define KQUADCOM_H

//device ids
enum { KQUAD_LL_DEVICE_ID,
       KQUAD_ML_DEVICE_ID,
       KQUAD_HL_DEVICE_ID,
       KQUAD_MOTOR_DEVICE_ID,
       KQUAD_SERVO1_DEVICE_ID,
       KQUAD_ZIGBEE_DEVICE_ID,
       KQUAD_PWR_DEVICE_ID
     };

#define KQUAD_LL_STD_CMD1  100
#define KQUAD_LL_STD_CMD2  101
#define KQUAD_LL_STD_CMD3  102
#define KQUAD_LL_STD_CMD4  103

#define KQUAD_LL_NANO_CMD1 125
#define KQUAD_LL_NANO_CMD2 126
#define KQUAD_LL_NANO_CMD3 127
#define KQUAD_LL_NANO_CMD4 128

enum {
       KQUAD_ZIGBEE_SET_MODE,
       KQUAD_ZIGBEE_SET_CHANNEL,
       KQUAD_ZIGBEE_ANALYZE_ED,
       KQUAD_ZIGBEE_RX_STATUS,
       KQUAD_ZIGBEE_CHK_ERROR
     };

#endif //KQUADCOM_H

