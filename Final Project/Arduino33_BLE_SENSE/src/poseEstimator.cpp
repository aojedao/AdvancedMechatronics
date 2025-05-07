#include "poseEstimator.hpp"
#include <math.h>
#include "config.h"

void updatePose(float &xPos, float &yPos, float &theta, Encoder &lEnc, Encoder &rEnc)
{
  static long deltaLeft = 0, deltaRight = 0;

  deltaLeft = lEnc.readAndReset();
  deltaRight = rEnc.readAndReset();
  

  float leftDist = (deltaLeft * 2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float rightDist = (deltaRight * 2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

  float dist = (leftDist + rightDist) / 2.0;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE;

  xPos += dist * cos(theta + dTheta / 2.0);
  yPos += dist * sin(theta + dTheta / 2.0);

  theta += dTheta;
  theta = atan2(sin(theta), cos(theta)); // Normalizer Theta between -pi and pi
}