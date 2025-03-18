#include <Udon.hpp>
#include <Wire.h>
#include "Suspension.hpp"

//ロボマス2つで足回り
//555を1つで破壊機構 R1
Udon::LoopCycleController loopCtrl{ 10000 };

Udon::Led led{ LED_BUILTIN };

Udon::CanBusTeensy<CAN1> bus;

Udon::E220PadPS5 pad({
    .serial = Serial2,
    .m0 = 2,
    .m1 = 3,
    .aux = 4,
  })

Suspension suspension
{
  std::array<RoboMasFB , 4>
  {
    RoboMasFB
    {
      Udon::RoboMasterC610{ bus , 1 }
      Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUS()}
    },
    RoboMaster::RoboMasFB
    {
      Udon::RoboMasterC610{ bus , 2 }
      Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUS()}
    }
  },
  Udon::PidController{ 0 , 0 , 0 , loopCtrl.cycleUS()},
  Gyro{ Udon::BNO055{ Wire } }
};

//555
Udon::CanWriter<Udon::Message::Motor> destroyer
{
  bus , 0x03
};

std::array<Udon::Message::Motor , 4> powers
{
  { Udon::Message::Motor{ ,power = 210 } },
  { Udon::Message::Motor{ ,power = -210 } },
  { Udon::Message::Motor{ ,power = 100 } },
  { Udon::Message::Motor{ ,power = -100 } }
};


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

  }
  else
  {
    suspension.stop();
  }

  loopCtrl.update();
}
