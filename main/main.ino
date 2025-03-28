#include <Udon.hpp>
#include "Suspension.hpp"
#include "Gyro.hpp"
#include "RoboMasFB.hpp"
#include "flag.hpp"

//ロボマス2つで足回り
//735を1つで破壊機構 R1
Udon::LoopCycleController loopCtrl{ 10000 };

Udon::Led led{ LED_BUILTIN };

Udon::CanBusTeensy<CAN1> comBus;
Udon::CanBusTeensy<CAN2> motorBus;

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
      Udon::RoboMasterC610{ motorBus , 1 },
      Udon::PidController{ 1 , 0 , 0.01 , loopCtrl.cycleUs() }
    },
    RoboMasFB
    {
      Udon::RoboMasterC610{ motorBus , 2 },
      Udon::PidController{ 1 , 0 , 0.01 , loopCtrl.cycleUs() }
    }
  },
  Udon::PidController{ 150 , 0.4 ,40 , loopCtrl.cycleUs()}, //Gyro
  Gyro{ Udon::BNO055{ Wire } }
};
double maxPower = 140;

//555
Udon::CanWriter<Udon::Message::Motor> destroyer{ comBus , 0x003 };

std::array<Udon::Message::Motor , 3> powers
{
  Udon::Message::Motor{ .power = 255, },
  Udon::Message::Motor{ .power = 30, },
  Udon::Message::Motor{ .power = -40, },
};

Flag flag;

void setup() 
{
  pad.begin(2);
  motorBus.begin();
  comBus.begin();
  led.begin();
  Serial.begin(115200);
  suspension.begin();
}

void loop() 
{
  led.flush();
  pad.update();
  motorBus.update();
  comBus.update();

  if(pad.isOperable())
  {
    // if(flag())
    // {
      if(!pad.getSquare().toggle)
      {
        const bool stick = suspension.StickMove();
        // Serial.println(suspension.StickMove());
        suspension.moveLikeOmni(pad.getMoveInfo() , maxPower , stick , pad.getL2().press);
        // Serial.println(suspension.LastAngle());
      }
      else
      {
        suspension.moveSuchStick( pad.getLeftStick().y , pad.getRightStick().y , maxPower , pad.getL2().press);
      }

      if(pad.getR1().press)
      {
        //Serial.println( 100 );
        destroyer.setMessage( powers[0] );
      }
      else if(pad.getL1().press)
      {
        //Serial.println( -100 );
        destroyer.setMessage( powers[1] );
      }
      else if(pad.getR2().press)
      {
        //Serial.println( 20 );
        destroyer.setMessage( powers[2] );
      }
      else 
      {
        //Serial.println( 0 );
        destroyer.setMessage( {0} );
      }

      if(pad.getTouch().click)
      {
        suspension.GyroClear();
      }

      // if(pad.getUp().click)
      // {
      //   suspension(false);
      //   flag(false);
      // }
    //}
    // else
    // {
    //   if(suspension.zeroPoint(suspension()))
    //   {
    //     flag(true);
    //   }
    // }
  }
  else
  {
    suspension.stop();
  }

  Udon::Show(pad.getMessage());
  Serial.println();
  loopCtrl.update();
}
