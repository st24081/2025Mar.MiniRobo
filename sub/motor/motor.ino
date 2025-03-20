#include <Udon.hpp>

Udon::CanBusSpi bus;

Udon::Led led{ LED_BUILTIN };

Udon::CanReader<Udon::Message::Motor> reader{ bus , 0x003 };

Udon::PicoWDT wdt;

Udon::Motor3 motor{ 8 , 10 , 9 };

void setup() 
{
  bus.begin();
  motor.begin();
  led.begin();
  Serial.begin(115200);
}

void loop() 
{
  wdt.update();
  bus.update();
  led.flush();

  if(const auto message = reader.getMessage())
  {
    int16_t power = message -> power;

    motor.move(power);
    Serial.print(power);
  }
  else
  {
    motor.stop();
  }

  if ((bool)bus) 
  {
    led.flush(500, 50);
  } 
  else 
  {
    led.flush(100, 30);
  }

  bus.show();
}
