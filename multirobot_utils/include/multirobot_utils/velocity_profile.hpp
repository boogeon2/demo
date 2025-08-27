#ifndef VELOCITY_PROFILE_HPP
#define VELOCITY_PROFILE_HPP
#include "cmath"

class VelocityProfile
{
public:
  // Parameters for the velocity profile
  double v_start;
  double v_max;
  double a_start;
  double dist;

  // Trapezoidal velocity profile modeled by the parameters
  double t_acc;
  double t_const;

  double dist_acc;
  double dist_const;

  // Constructor
  VelocityProfile() {};

  VelocityProfile(
      double v_start_,
      double v_max_,
      double a_start_,
      double dist_);

  // Functions
  void make_vel_prof();
  double get_cost();
  double calc_dist(double t);
  double calc_time(double dist);
};

#endif
