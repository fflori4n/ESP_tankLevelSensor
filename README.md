# ESP_tankLevelSensor

ESP32 measuring water level in a reservoar, using an external LM555 oscillator and a capacitive probe. 

Measurements are displayed on MAX7219 driven 7seg. displays and the uController hosts it's on webserver that displays the tank status and stats. Board has a linear regulator to drop the 12V DC from the truck's battery to 5V for the ESP and the sensor, two relays that can switch 12V DC external loads ( meant for signal lights ), and can drive WS2812B LED strip to indicate level in the tank.
