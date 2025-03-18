#pragma once

#include <Udon.hpp>


  class RoboMasFB
  {
    Udon::RoboMasterC610 motor;
    Udon::PidController pid;

    public :
    RoboMasFB(Udon::RoboMasterC610&& motor , Udon::PidController&& pid)
    : motor{ std::move(motor) }
    , pid{ std::move(pid) }
    {
    }

    void move(const double targetVelocity)
    {
      const auto current = pid(motor.getVelocity(), targetVelocity);
      motor.setCurrent(current);
    }
 
    void stop()
    {
      pid.clearPower();
      motor.setCurrent(0);
    }
  };
