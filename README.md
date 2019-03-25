# Set Up
##  Required software:
- Arduino IDE 1.8.5
- Teensyduino plugin 1.40
- Newliquidcrystal 1.3.5 https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/Newliquidcrystal_1.3.5.zip

## Step by step setup instuctions:
- Install Arduino IDE 1.8.8
- Install Teensyduino plugin 1.45
- Open sketch file in Arduino IDE
- Download Newliquidcrystal
- Add Newliquidcrystal library: Sketch -> Include library -> Add .zip library
- Select Board: Tools -> Board -> Teensy LC
- Select Tools -> USB Type -> Serial
- Select serial port: Tools -> Port -> COMx (/dev/ttyACMx for Linux)
- Compile sketch: Sketch -> Verify/Compile
- Upload sketch: Sketch -> Upload

- Make sure UART Modem is in unprovisioned state (perform factory reset)
- Now you should be able to add device to space with iOS Platform app

## Troubleshooting tips:
- Make sure you have proper serial port selected in Arduino IDE: Tools -> Port -> COMx
- You can read debug console in Arduino IDE: Tools -> Serial Monitor (115200 baud)
