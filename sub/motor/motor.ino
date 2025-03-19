#include <Udon.hpp>

Udon::CanBusSpi bus;

Udon::Led led{ LED_BUILTIN };

Udon::CanReader<Udon::Message::Motor> reader{ bus , 0x003 };

Udon::Motor3 motor{ 3 , 5 , 4 };

void setup() 
{
  bus.begin();
  motor.begin();
  led.begin();
  Serial.begin(115200);
}

void loop() 
{
  bus.update();
  led.flush();

  if(const auto message = reader.getMessage())
  {
    int16_t power = message -> power;

    motor.move(power);
    Serial.println(power);
  }
  else
  {
    motor.stop();
  }
}
