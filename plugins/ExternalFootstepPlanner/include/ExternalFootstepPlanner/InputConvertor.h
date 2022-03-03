#pragma once

#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <ExternalFootstepPlanner/SE2d.h>
#include <atomic>
#include <map>
#include <math.h>
#include <mutex>
#include <random>
// #include <thread>
#include <vector>

namespace mc_plugin
{
namespace ExternalFootstepPlanner
{

class InputConvertor
{
public:
  InputConvertor() = delete;
  ~InputConvertor() = delete;

  static SE2d convert_PS4_to_SE2d(const sensor_msgs::Joy & joy_msg);
  static SE2d convert_Oculus_to_SE2d(const sensor_msgs::Joy & joy_msg);
  static double L2norm(double a_x, double a_y, double b_x = 0.0, double b_y = 0.0); // calc L2 distance between A and B
  static double Uniform_norm(double a_x, double a_y);
  static double calc_exponential_interpolation(double exp, double y_max, double x);
};

} // namespace ExternalFootstepPlanner
} // namespace mc_plugin