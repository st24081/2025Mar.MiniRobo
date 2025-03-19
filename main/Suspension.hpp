#pragma once

#include <algorithm>
#include "Gyro.hpp"
#include "RoboMasFB.hpp"

class Suspension
{
  std::array<RoboMasFB , 2> motors;
  Udon::PidController pidGyro;
  Gyro gyro;
  uint turnTime;
  uint zeroTime;
  bool T;
  
  public :
  Suspension(std::array<RoboMasFB , 2>&& motors , Udon::PidController&& pidGyro , Gyro&& gyro)
  : motors{ std::move(motors) }
  , pidGyro{ std::move(pidGyro) }
  , gyro{ std::move(gyro) }
  , turnTime(0)
  , zeroTime(0)
  , T(false)
  {
  }

  void begin() 
  {
    gyro.begin();
  }

  void moveLikeOmni(Udon::Stick moveInfo, double maxPower)
  {
    gyro.update();
    Udon::Vec2 maxVec{ 255 , 255 };
    double length = map(moveInfo.vector.length() , -maxVec.length() , maxVec.length() , -255 , 255);

    double stickAngle = 0;
    if(moveInfo.vector.x || moveInfo.vector.y )
    {
      stickAngle = moveInfo.vector.angle();
    }

    moveInfo *= ( maxPower / 255 );
    if (moveInfo.turn) 
    {
      turnTime = millis();
    }
    if (millis() - turnTime < 500) 
    {
      gyro.clear();
    }

    Serial.println(gyro.getAngle() * -1);
    pidGyro.update(gyro.getAngle() * -1, stickAngle);//怖い
    moveInfo.turn -= pidGyro.getPower();
    Serial.println(stickAngle);

    double leftMove = length + moveInfo.turn;
    double rightMove = length + moveInfo.turn;

    auto&& maxP = abs(max(leftMove, rightMove));
    if(maxP > maxPower)
    {
      const auto ratio = maxPower / maxP;
      leftMove *= ratio;
      rightMove *= ratio;
    }

    //moveInfo = map(moveInfo , -255 , 255 , -10000 , 10000 );
    leftMove = map(leftMove , -255 , 255 , -10000 , 10000 );
    rightMove = map(rightMove , -255 , 255 , -10000 , 10000 );

    motors[0].move( leftMove );
    motors[1].move( rightMove );
  }

  void moveSuchStick(int leftY , int rightY , double maxPower)
  {
    leftY *= (maxPower / 255 );
    rightY  *= (maxPower / 255 );
    leftY = map( leftY , -255 , 255 , -10000 , 10000 );
    rightY = map( rightY , -255 , 255 , -10000 , 10000 );

    motors[0].move(leftY);
    motors[1].move(rightY);
  }

  void stop() 
  {
    for (auto& motor : motors) 
    {
      motor.stop();
    }
    pidGyro.clearPower();
    gyro.clear();
  }

  bool zeroPoint(bool t)
  {
    if(!t)
    {
      zeroTime = millis();
      T = true;
    }
    if(millis() - zeroTime >= 2000)
    {
      return true;
    }
    else
    {
      for(auto& motor : motors) 
      {
        motor.move( - pidGyro(gyro.getAngle() , 0));
      }
      return false;
    }
  }

  bool operator()()
  {
    return T;
  }
  void operator()(bool t)
  {
    T = t;
  }
};
