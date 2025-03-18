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

  void moveLikeOmni(Udon::Stick moveInfo, Udon::Vec2 LeftStick , double maxPower)
  {
    moveInfo *= ( maxPower / 255 );
    if (moveInfo.turn) 
    {
      turnTime = millis();
    }
    if (millis() - turnTime < 500) 
    {
      gyro.clear();  
    }
    
    double stickAngle = 0;
    if(LeftStick.x || LeftStick.y )
    {
      stickAngle = LeftStick.angle();
    }
    //下のgetAngleをいじるか新しい関数を作って疑似ポジティブ制御を行いたい
    moveInfo.turn -= pidGyro(gyro.getAngle() , stickAngle);//怖い

    double leftMove = moveInfo.vector.y + moveInfo.turn;
    double rightMove = -moveInfo.vector.y + moveInfo.turn;

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
    if(millis() - zeroTime > 2000)
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
