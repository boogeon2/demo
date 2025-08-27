#include "multirobot_utils/velocity_profile.hpp"

VelocityProfile::VelocityProfile(
    double v_start_,
    double v_max_,
    double a_start_,
    double dist_)
{
  v_start = v_start_;
  v_max = v_max_;
  a_start = a_start_;
  dist = dist_;
  make_vel_prof();
}

void VelocityProfile::make_vel_prof()
{
  dist_const = dist - (v_max + v_start) * ((v_max - v_start) / a_start) / 2;

  if (dist_const <= 0) // If the distance is too short to reach v_max
  {
    t_acc = sqrt(2 * dist / a_start);
    t_const = 0;
    dist_acc = dist;
    dist_const = 0.0;
  }
  else // If the distance is long enough to reach v_max
  {
    t_acc = (v_max - v_start) / a_start;
    dist_acc = (v_start + v_max) * t_acc / 2;
    dist_const = dist - dist_acc;
    t_const = dist_const / v_max;
  }
}

double VelocityProfile::get_cost()
{
  return t_acc + t_const;
}

double VelocityProfile::calc_dist(double t) // Calculate the dist at time t(relative time)
{
  if (t < t_acc) // Acceleration phase
  {
    return v_start * t + 0.5 * a_start * t * t;
  }
  else if (t < t_acc + t_const) // Constant velocity phase
  {
    return dist_acc + v_max * (t - t_acc);
  }
  else // Impossible case
  {
    return 0.0;
  }
}

double VelocityProfile::calc_time(double dist) // Calculate the time at distance dist
{
  if (dist < dist_acc) // Acceleration phase
  {
    return (-v_start + sqrt(v_start * v_start + 2 * a_start * dist)) / a_start;
  }
  else if (dist < dist_acc + dist_const) // Constant velocity phase
  {
    return t_acc + (dist - dist_acc) / v_max;
  }
  else // Impossible case
  {
    return 0.0;
  }
}
