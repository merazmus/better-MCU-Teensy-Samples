# Lightness Client

## Usage
There are three buttons, a potentiometer and an encoder used in this example. Each has a different function:
- **PB0**	Trigger sending Generic On Off message with value 1. Turns the light on.
- **PB1**	Trigger sending Generic On Off message with value 0. Turns the light off.
- **PB4**	Trigger sending Light Lightness Controller Mode Set message with value 1. Turns controller mode on.
- **Potentiometer**	Changes trigger sending Light Lightness Set message with value proportional to voltage level on PIN_ANALOG. Message is sent in minimal interval of LIGHTNESS_INTVL_MS.
- **Encoder** Trigger sending Generic Delta message with value proportional to encoder rotation since new transaction started.
