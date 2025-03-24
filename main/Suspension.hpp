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
  bool stickMove;
  double lastAngle;
  
  public :
  Suspension(std::array<RoboMasFB , 2>&& motors , Udon::PidController&& pidGyro , Gyro&& gyro)
  : motors{ std::move(motors) }
  , pidGyro{ std::move(pidGyro) }
  , gyro{ std::move(gyro) }
  , turnTime(0)
  , stickMove(true)
  , lastAngle(0)
  {
  }

  void begin() 
  {
    gyro.begin();
  }

  void moveLikeOmni(Udon::Stick moveInfo, double limitPower , bool stick , bool L2)
  {
    gyro.update();
    double length = moveInfo.vector.length();
    length = map(length , -300 , 300 , -150 , 150);
    Serial.println(lastAngle);

    double stickAngle = 0;
    // if(!L2)
    // {
      if(moveInfo.vector.x || moveInfo.vector.y )
      {
        stickAngle = moveInfo.vector.angle();
        stickMove = true;
      }
      else
      {
        stickMove = false;
      }
    // }
    // else
    // {
    //   stickAngle = Udon::Pi;
    //   moveInfo.vector *= -1; 
    //   stickAngle = moveInfo.vector.angle();
    // }

    if (moveInfo.turn) 
    {
      turnTime = millis();
    }
    if (millis() - turnTime < 500) 
    {
      //gyro.clear();
      stickAngle = gyro.getAngle();
    }

    Serial.println(gyro.getAngle() * -1);
    // if(stick)
    // {
      if(abs(stickAngle - gyro.getAngle() * -1) <= Udon::Pi / 2 || abs(stickAngle - gyro.getAngle() * -1) >= 3 * Udon::Pi / 2)
      {
        pidGyro.update(gyro.getAngle() * -1, stickAngle);//怖い
        if(stickAngle != 0)
        {
          lastAngle = stickAngle;
        }
        Serial.println("A");
      }
      else
      { 
        if(stickAngle > 0)
        {
          if(gyro.getAngle() * -1 <= Udon::Pi / 2)
          {
            pidGyro.update(gyro.getAngle() * -1, stickAngle - Udon::Pi);
            lastAngle = stickAngle - Udon::Pi;
            Serial.println("B.A.A");
          }
          else
          {
            pidGyro.update(gyro.getAngle() * -1, stickAngle + Udon::Pi);
            lastAngle = stickAngle + Udon::Pi;
            Serial.println("B.A.B");
          }
        
        }
        else if(stickAngle < 0)
        {
          if(gyro.getAngle() * -1 >= - Udon::Pi / 2)
          {
            pidGyro.update(gyro.getAngle() * -1, stickAngle + Udon::Pi);
            lastAngle = stickAngle + Udon::Pi;
            Serial.println("B.B.A");
          }
          else
          {
            pidGyro.update(gyro.getAngle() * -1, stickAngle - Udon::Pi);
            lastAngle = stickAngle - Udon::Pi;
            Serial.println("B.B.B");
          }
        }
        else
        {
          if(gyro.getAngle() * -1 < 0)
          {
            pidGyro.update(gyro.getAngle() * -1, -Udon::Pi );
            lastAngle = -Udon::Pi;
          }
          else
          {
            pidGyro.update(gyro.getAngle() * -1, Udon::Pi );
            lastAngle = Udon::Pi;
          }
          Serial.println("B.C");
        }

        length *= -1;
      }
    // }
    // else if(moveInfo.turn)
    // {
    //   pidGyro.clearPower();
    // }
    // else
    // {
    //   pidGyro.update(gyro.getAngle() , lastAngle);
    // }

    Serial.println(lastAngle);
    moveInfo.turn += pidGyro.getPower();
    Serial.println(stickAngle);
    
    double leftMove = 0;
    double rightMove = 0;

    // if(!L2)
    // {
    //   leftMove = length + moveInfo.turn;
    //   rightMove = -length + moveInfo.turn;
    // }
    // else
    // {
      leftMove = length + moveInfo.turn;
      rightMove = -length + moveInfo.turn;
    // }

    // ///////////////
    // if (moveInfo.turn) 
    // {
    //   turnTime = millis();
    // }
    // if (millis() - turnTime < 500) 
    // {
    //   gyro.clear();
    //   // stickAngle = gyro.getAngle();
    // }
    // Serial.println(gyro.getAngle() * -1);
    // pidGyro.update(gyro.getAngle() , 0);
    // moveInfo.turn -= pidGyro.getPower();
    // double leftMove = moveInfo.vector.y + moveInfo.turn;
    // double rightMove = -moveInfo.vector.y + moveInfo.turn;
    // if(L2)
    // {
    //   leftMove *= -1;
    //   rightMove *= -1;
    // }


    // Serial.println(leftMove);
    // Serial.println(rightMove);

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

  void moveSuchStick(int leftY , int rightY , double maxPower , bool L2)
  {
    leftY *= (maxPower / 255 );
    rightY  *= (maxPower / 255 );
    leftY = map( leftY , -255 , 255 , -10000 , 10000 );
    rightY = map( rightY , -255 , 255 , -10000 , 10000 );

    rightY *= -1;

    if(L2)
    {
      leftY *= -1;
      rightY *= -1;
    }

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
    gyro.clear();
  }

  void GyroClear()
  {
    gyro.clear();
  }

  bool StickMove()
  {
    return stickMove;
  }

  double LastAngle()
  {
    return lastAngle;
  }
};
