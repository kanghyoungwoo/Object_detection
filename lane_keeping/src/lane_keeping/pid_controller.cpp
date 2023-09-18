/**
 * @file pid_controller.cpp
 * @author Jiho Han (hanjiho97@naver.com)
 * @brief PID Controller Class source file
 * @version 0.3
 * @date 2023-01-20
 */
#include "lane_keeping_system/pid_controller.h"
namespace xycar
{
PID::PID(float p_gain, float i_gain, float d_gain)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain)
{
  p_error_ = 0.0F;
  i_error_ = 0.0F;
  d_error_ = 0.0F;
}

float PID::getAngle(float steering_angle)
{
  current_angle = steering_angle;
}

float PID::getControlOutput(int error)
{
  if (current_angle < 20.0F)
  {
    p_gain_ = 0.30F;
    i_gain_ = 0.000004F;
    d_gain_ = 0.00F;
  }
  else
  {
    p_gain_ = 0.37F;
    i_gain_ = 0.000025F;
    d_gain_ = 0.00F;
  }

  float float_type_error = (float)error;
  if (error == 0)
  {
    i_error_ = 0;
  }
  else
  {
    i_error_ += float_type_error;
  }
  p_error_ = float_type_error;
  d_error_ = float_type_error - p_error_;
  return p_gain_ * p_error_ + i_gain_ * i_error_ + d_gain_ * d_error_;
}
}  // namespace xycar
