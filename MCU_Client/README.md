# Lightness and Sensor Client

Lightness Client act as a switch. You can use it control your space.
Sensor Client displays received measurements on LCD.

## Usage
There are four buttons, a potentiometer and an encoder used in this example. Each has a different function:
- **SW1**   Trigger sending Generic On Off message with value 1. Turns the light on.
- **SW2**   Trigger sending Generic On Off message with value 0. Turns the light off.
- **SW4**   Reinitialize LCD if needed.
- **Encoder switch**   Trigger sending Light Lightness Controller Mode Set message with value 1. Turns controller mode on.
- **Potentiometer** Changes trigger sending Light Lightness Set message with a value proportional to the voltage level on PIN_ANALOG. The message is sent in a minimal interval of LIGHTNESS_INTVL_MS.
- **Encoder** Trigger sending Generic Delta message with the value proportional to encoder rotation since new transaction started.
