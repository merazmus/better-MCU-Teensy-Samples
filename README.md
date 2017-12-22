# Set Up
##  Required software:
- Arduino IDE 1.8.5
- Teensyduino plugin 1.40

## Step by step setup instuctions:
- Install Arduino IDE 1.8.5
- Install Teensyduino plugin 1.40
- Open sketch file in Arduino IDE
- Select Board: Tools -> Board -> Teensy LC
- Select Tools -> USB Type -> Serial
- Select serial port: Tools -> Port -> COMx (/dev/ttyACMx for Linux)
- Compile sketch: Sketch -> Verify/Compile
- Upload sketch: Sketch -> Upload

- Make sure UART Modem is in unprovisioned state (perform factory reset)
- Now you should be able to add device to space with iOS Platform app

## Troubleshooting tips:
- Make sure you have proper serial port selected in Arduino IDE: Tools -> Port -> COMx
- You can read debug console in Arduino IDE: Tools -> Serial Monitor (11520 baud)
