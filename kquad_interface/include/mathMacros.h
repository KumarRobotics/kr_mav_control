/* Copyright KMel Robotics 2012. Must read KMEL_LICENSE.pdf for terms and conditions before use. */
#ifndef MATH_MACROS_H
#define MATH_MACROS_H

#define CONSTRAIN_MIN_MAX(var,minVal,maxVal) \
{\
  if ((var) > (maxVal)) var = maxVal; \
  else if ((var) < (minVal)) var = minVal; \
}

#define CONSTRAIN_MIN(var,minVal) \
{\
  if ((var) < (minVal)) var = minVal; \
}

#define CONSTRAIN_MAX(var,maxVal) \
{\
  if ((var) > (maxVal)) var = maxVal; \
}

#define D2R(deg) (deg/180.0*M_PI)
#define R2D(rad) (rad*180.0/M_PI)

#define M_2PI (2.0*M_PI)

inline static float Mod2PiF(float val)
{
  if (val > M_PI)
  {
    float y1 = val + M_PI;
    int   y2 = y1  / M_2PI;
    val      = y1  - (y2 * M_2PI) - M_PI;
  }
  else if (val < -M_PI)
  {
    float y1 = val - M_PI;
    int   y2 = y1  / M_2PI;
    val     = y1  - (y2 * M_2PI) + M_PI;
  }

  return val;
}

inline static double Mod2PiD(double val)
{
  if (val > M_PI)
  {
    double y1 = val + M_PI;
    int    y2 = y1  / M_2PI;
    val       = y1  - (y2 * M_2PI) - M_PI;
  }
  else if (val < -M_PI)
  {
    double y1 = val - M_PI;
    int    y2 = y1  / M_2PI;
    val      = y1  - (y2 * M_2PI) + M_PI;
  }

  return val;
}

#endif //MATH_MACROS_H

