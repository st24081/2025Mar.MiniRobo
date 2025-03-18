#include <Udon.hpp>
#include <Wire.h>
#include "Suspension.hpp"
#include "Gyro.hpp"
#include "RoboMasFB.hpp"
#include "flag.hpp"

//ロボマス2つで足回り
//555を1つで破壊機構 R1
Udon::LoopCycleController loopCtrl{ 10000 };

Udon::Led led{ LED_BUILTIN };

Udon::CanBusTeensy<CAN1> bus;

Udon::E220PadPS5 pad
  ({
    .serial = Serial2,
    .m0 = 2,
    .m1 = 3,
    .aux = 4,
  });

Suspension suspension
{
  std::array<RoboMasFB , 2>
  {
    RoboMasFB
    {
      Udon::RoboMasterC610{ bus , 1 },
      Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUs() }
    },
    RoboMasFB
    {
      Udon::RoboMasterC610{ bus , 2 },
      Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUs() }
    }
  },
  Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUs() }, //Gyro
  Gyro{ Udon::BNO055{ Wire } }
};
double maxPower = 100;

//555
Udon::CanWriter<Udon::Message::Motor> destroyer
{
  bus , 0x003
};

std::array<Udon::Message::Motor , 4> powers
{
  Udon::Message::Motor{ .power = 210, },
  Udon::Message::Motor{ .power = -210, },
  Udon::Message::Motor{ .power = 100, },
  Udon::Message::Motor{ .power = -100, }
};

Flag flag;

void setup() 
{
  pad.begin();
  bus.begin();
  led.begin();
  Serial.begin(115200);
}

void loop() 
{
  led.flush();
  pad.update();
  bus.update();

  if(pad.isOperable())
  {
    if(flag())
    {
      if(!pad.getSquare().toggle)
      {
        suspension.moveLikeOmni(pad.getMoveInfo() , pad.getLeftStick() , maxPower);
      }
      else
      {
        suspension.moveSuchStick( pad.getLeftStick().y , pad.getRightStick().y , maxPower );
      }

      if(pad.getR1().press)
      {
        destroyer.setMessage( powers[0] );
      }
      else if(pad.getL1().press)
      {
        destroyer.setMessage( powers[1] );
      }
      else if(pad.getR2().press)
      {
        destroyer.setMessage( powers[2] );
      }
      else if(pad.getL2().press)
      {
        destroyer.setMessage( powers[3] );
      }

      if(pad.getUp().click)
      {
        suspension(false);
        flag(false);
      }
    }
    else
    {
      if(suspension.zeroPoint(suspension()))
      {
        flag(true);
      }
    }
  }
  else
  {
    suspension.stop();
  }

  loopCtrl.update();
}
