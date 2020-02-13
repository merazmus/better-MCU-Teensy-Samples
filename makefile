BUILD_PARAMS    = --verbose --verify --useprogrammer --board "teensy:avr:teensyLC:speed=48,usb=serial,keys=en-gb,opt=osstd"

.PHONY: MCU_Client MCU_Server clean

clean:
	rm -rf _build_client
	rm -rf _build_server

MCU_Server:
	/opt/arduino-1.8.8/arduino $(BUILD_PARAMS) MCU_Server/*.ino --pref build.path=_build_MCU_Server/

MCU_Client:
	/opt/arduino-1.8.8/arduino $(BUILD_PARAMS) MCU_Client/*.ino --pref build.path=_build_MCU_Client/
