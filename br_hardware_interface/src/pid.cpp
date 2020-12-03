//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/pid.hpp"

double PID::update(
  double error, std::chrono::steady_clock::duration dt,
  double & p_term, double & i_term, double & d_term)
{
  p_term = p_ * error;
  if (use_anti_windup_) {
    /// If error passed zero (i.e. last and now have different signs) set the integral to zero
    if (last_error_ * error < 0 || error == 0) {
      integral_ = 0;
    }
  }
  integral_ = std::min(std::max(integral_ + error * (dt.count() / 1000000000.0), min_i_), max_i_);
  i_term = i_ * integral_;
  d_term = d_ * (error - last_error_) / (dt.count() / 1000000000.0);
  last_error_ = error;

  auto output = std::min(std::max(last_cmd_ + (p_term + i_term + d_term), -1.0), 1.0);
  last_cmd_ = output;
  return output;
}

void PID::set_p(double p)
{
  p_ = p;
}

void PID::set_i(double i)
{
  i_ = i;
}

void PID::set_d(double d)
{
  d_ = d;
}

void PID::set_i_clamp_max(double max_i)
{
  max_i_ = max_i;
}

void PID::set_i_clamp_min(double min_i)
{
  min_i_ = min_i;
}

void PID::set_anti_windup(bool use_anti_windup)
{
  use_anti_windup_ = use_anti_windup;
}
