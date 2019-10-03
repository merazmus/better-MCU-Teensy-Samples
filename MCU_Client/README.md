# Lightness and Sensor Client

## IMPORTANT!
'Serial2' class from Teensyduino library is not supported (project has own implementation of UART1 based on DMA). Using
it may cause application problems.

## Usage
There are four buttons, a potentiometer and an encoder used in this example. Each has a different function:
- **SW1**   Trigger sending Generic On Off message with value 1.
- **SW2**   Trigger sending Generic On Off message with value 0.
- **SW3**   Trigger sending Generic On Off message with value 1.
- **SW4**   Trigger sending Generic On Off message with value 0.
- **Potentiometer** Changes trigger sending Generic Level Set message with a value proportional to the voltage level on PIN_ANALOG. The message is sent in a minimal interval of LIGHTNESS_INTVL_MS.
- **Encoder** Trigger sending Generic Delta Set message with the value proportional to encoder rotation since new transaction started.
