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

  void moveLikeOmni(Udon::Stick moveInfo, double limitPower)
  {
    gyro.update();
    double length = moveInfo.vector.length();
    Serial.println(length);

    double stickAngle = 0;
    if(moveInfo.vector.x || moveInfo.vector.y )
    {
      stickAngle = moveInfo.vector.angle();
    }

    if (moveInfo.turn) 
    {
      turnTime = millis();
    }
    if (millis() - turnTime < 500) 
    {
      gyro.clear();
      //stickAngle = gyro.getAngle();
    }

    Serial.println(gyro.getAngle() * -1);
    // if(abs(stickAngle - gyro.getAngle()) <= abs(Udon::Pi / 2))
    // {
      pidGyro.update(gyro.getAngle() * -1, stickAngle);//怖い
    // }
    // else
    // {
    //   if(abs(stickAngle) < Udon::Pi / 2)
    //   {
    //     if(stickAngle > 0)
    //     {
    //       // pidGyro.update(gyro.getAngle() * -1, stickAngle * -1 -2( Udon::Pi / 2 + stickAngle));
    //       pidGyro.update(gyro.getAngle() * -1, stickAngle * -1 + 2*( Udon::Pi / 2 + stickAngle));
    //     }
    //     else if(stickAngle < 0)
    //     {
    //       //pidGyro.update(gyro.getAngle() * -1, stickAngle * -1 -2( Udon::Pi / 2 - stickAngle));
    //       pidGyro.update(gyro.getAngle() * -1, stickAngle * -1 - 2*( Udon::Pi / 2 + stickAngle));
    //     }
    //     else
    //     {
    //       pidGyro.update(gyro.getAngle() * -1, 0 );
    //     }
    //   }
    //   else if(abs(stickAngle) > Udon::Pi / 2);
    //   {
    //     if(stickAngle > 0)
    //     {
    //       pidGyro.update(gyro.getAngle() * -1, (stickAngle + 2*(Udon::Pi / 2 + ( -Udon::Pi - stickAngle))) * -1);
    //     }
    //     else if(stickAngle < 0)
    //     {
    //       pidGyro.update(gyro.getAngle() * -1, (stickAngle - 2*(Udon::Pi / 2 - (Udon::Pi - stickAngle))) * -1);
    //     }
    //   }
    // }
    moveInfo.turn -= pidGyro.getPower();
    Serial.println(stickAngle);

    double leftMove = -length + moveInfo.turn;
    double rightMove = length + moveInfo.turn;

    Serial.println(leftMove);
    Serial.println(rightMove);

    // const double maxPower = max(abs(leftMove,rightMove));
    // if (maxPower > limitPower)
    // {
    //   const double ratio = limitPower / maxPower;
    //   leftMove *= ratio;
    //   rightMove *= ratio;
    // }

    leftMove = map(leftMove , -255 , 255 , -limitPower , limitPower );
    rightMove = map(rightMove , -255 , 255 , -limitPower , limitPower );

    //moveInfo = map(moveInfo , -255 , 255 , -10000 , 10000 );
    leftMove = map(leftMove , -255 , 255 , -10000 , 10000 );
    rightMove = map(rightMove , -255 , 255 , -10000 , 10000 );

    Serial.println(leftMove);
    Serial.println(rightMove);

    motors[0].move( leftMove );
    motors[1].move( rightMove );
  }

  void moveSuchStick(int leftY , int rightY , double maxPower)
  {
    leftY *= (maxPower / 255 );
    rightY  *= (maxPower / 255 );
    leftY = map( leftY , -255 , 255 , -10000 , 10000 );
    rightY = map( rightY , -255 , 255 , -10000 , 10000 );

    Serial.println(leftY);
    Serial.println(rightY);
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
