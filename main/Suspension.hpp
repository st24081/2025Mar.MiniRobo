#pragma once

#include <algorithm>
#include "Gyro.hpp"
#include "RoboMasFB.hpp"

class Suspension
{
  std::array<RoboMaster::RoboMasFB , 2> motors;
  Udon::PidController pid;
  Gyro gyro;
  uint turnTime;
  
  public :
  Suspension(std::array<RoboMaster::RoboMasFB , 2>&& motors , Udon::PidController&& pid , Gyro&& gyro)
  : motors{ std::move(motors) }
  , pid{ std::move(pid) }
  , gyro{ std::move(gyro) }
  , turnTime(0)
  {
  }

  void begin() 
  {
    gyro.begin();
  }

  void moveLikeOmni(Udon::Stick moveInfo, double maxCurrent)
  {
    moveInfo *= ( 10000 / 255 );
    moveInfo *= (maxCurrent / 10000 );
    if (moveInfo.turn) 
    {
      turnTime = millis();
    }
    if (millis() - turnTime < 500) 
    {
      gyro.clear();  
    }
    moveInfo.turn -= pid(gyro.getAngle() , 0);

    auto maxPower = abs(max(moveInfo.vector.y + moveInfo.turn, -moveInfo.vector.y + moveInfo.turn));
    if(maxPower >= maxCurrent )
    {
      const auto ratio = maxCurrent / abs(moveInfo.vector.y + moveInfo.turn);
      moveInfo *= ratio;
    }

    motors[0].move( moveInfo.vector.y + moveInfo.turn );
    motors[1].move( -moveInfo.vector.y + moveInfo.turn );
  }

  void moveSuchStick(int leftY , int rightY , double maxCurrent)
  {
    leftY *= (10000 / 255);
    rightY *= (10000 / 255);
    leftY *= (maxCurrent / 10000 );
    rightY  *= (maxCurrent / 10000 );

    motors[0].move(leftY);
    motors[1].move(rightY);
  }

  void stop() 
  {
    for (auto& motor : motors) 
    {
      motor.stop();
    }
    pid.clearPower();
    gyro.clear();
  }
  
};
