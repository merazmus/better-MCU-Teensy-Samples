/*
 * UART_Client_Sample.ino
 *
 * Created: 9/5/2017 9:56:43 AM
 * Author: Maciej WitaliĹ„ski
 */

#include <EEPROM.h>

/*************************************************************************
 *
 * Definitions
 *
 *************************************************************************/

#define PIN_ENCODER_A               5
#define PIN_ENCODER_B               3
#define PIN_ENCODER_CLICK           14
#define PIN_LED                     16
#define PIN_PB_0                    22
#define PIN_PB_1                    21
#define PIN_PB_2                    20
#define PIN_PB_3                    15

#define PB_ON                       PIN_PB_0
#define PB_OFF                      PIN_PB_1
#define PB_DELTA_UP                 PIN_PB_2
#define PB_DELTA_DOWN               PIN_PB_3

#define LIGHTNESS_MIN               0
#define LIGHTNESS_MAX               UINT16_MAX
#define LIGHTNESS_STEP              0x1000

#define LIGHT_LC_MODE_OFF           0x00
#define LIGHT_LC_MODE_ON            0x01

#define GENERIC_ON_OFF_OFF          0x00
#define GENERIC_ON_OFF_ON           0x01

#define ENCODER_6BIT_MASK           0x3F
#define ENCODER_RIGHT_TURN_PATTERN  0b000111
#define ENCODER_LEFT_TURN_PATTERN   0b001011

#define BUTTON_DEBOUNCE_TIME_MS     20

#define GENERIC_DELTA_UP_VALUE      (0x1000)
#define GENERIC_DELTA_DOWN_VALUE    (-0x1000)

#define UART_CMD_PING               0x01
#define UART_CMD_PONG               0x02
#define UART_CMD_AVAILABLE_MODELS   0x03
#define UART_CMD_SET_INSTANCES      0x04
#define UART_CMD_CREATED_INSTANCES  0x05
#define UART_CMD_SYNC_START         0x06
#define UART_CMD_MESH               0x07
#define UART_CMD_OPCODE_ERROR       0x08
#define UART_CMD_SYNC_FINISH        0x09

#define MESH_MODEL_ID_GENERIC_ONOFF_CLIENT                0x1001
#define MESH_MESSAGE_GENERIC_ONOFF_GET                    0x8201
#define MESH_MESSAGE_GENERIC_ONOFF_SET                    0x8202
#define MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED     0x8203
#define MESH_MESSAGE_GENERIC_ONOFF_STATUS                 0x8204

#define MESH_MODEL_ID_GENERIC_LEVEL_CLIENT                0x1003
#define MESH_MESSAGE_GENERIC_LEVEL_GET                    0x8205
#define MESH_MESSAGE_GENERIC_LEVEL_SET                    0x8206
#define MESH_MESSAGE_GENERIC_LEVEL_SET_UNACKNOWLEDGED     0x8207
#define MESH_MESSAGE_GENERIC_LEVEL_STATUS                 0x8208
#define MESH_MESSAGE_GENERIC_DELTA_SET                    0x8209
#define MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED     0x820A
#define MESH_MESSAGE_GENERIC_MOVE_SET                     0x820B
#define MESH_MESSAGE_GENERIC_MOVE_SET_UNACKNOWLEDGED      0x820C

#define MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT            0x1008
#define MESH_MESSAGE_GENERIC_ONPOWERUP_GET                0x8211
#define MESH_MESSAGE_GENERIC_ONPOWERUP_SET                0x8213
#define MESH_MESSAGE_GENERIC_ONPOWERUP_SET_UNACKNOWLEDGED 0x8214
#define MESH_MESSAGE_GENERIC_ONPOWERUP_STATUS             0x8212

#define MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT              0x1302
#define MESH_MESSAGE_LIGHT_LIGHTNESS_GET                  0x824B
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET                  0x824C
#define MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED   0x824D
#define MESH_MESSAGE_LIGHT_LIGHTNESS_STATUS               0x824E
#define MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT                   0x1311
#define MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_GET                  0x8291
#define MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_SET                  0x8292
#define MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_SET_UNACKNOWLEDGED   0x8293
#define MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_STATUS               0x8294

#define CRC_POLYNOMIAL        0x8005

#define PREAMBLE_BYTE_1       0xAA
#define PREAMBLE_BYTE_2       0x55

struct RxFrame
{
  uint8_t     Len;
  uint8_t     Cmd;
  uint8_t     Buf[20];
  uint8_t     Cnt;
} RxFrame;

/*************************************************************************
 *
 * Global variables
 *
 *************************************************************************/

volatile int32_t lightnessActual = LIGHTNESS_MIN;
volatile int32_t lightnessLast   = LIGHTNESS_MAX; // last non-zero value of Lightness
volatile bool deltaUp = false;
volatile bool deltaDown = false;
volatile bool lightLcMode = false;
volatile bool on = false;
volatile bool off = false;
/*************************************************************************
 *
 * Main functions
 *
 *************************************************************************/

void setup()
{
  SetupDebug();
  SetupUART();
  SetupEncoder();
  SetupPB();
}

void loop()
{
  UARTCommandProcess();

  if(lightLcMode)
  {
    lightLcMode = false;
    PrintMode();
    SendLightLightnessControllerModeSet(LIGHT_LC_MODE_ON);
  }
  
  if (deltaUp)
  {
    deltaUp = false;
    SendGenDeltaSet(GENERIC_DELTA_UP_VALUE);
  }

  if (deltaDown)
  {
    deltaDown = false;
    SendGenDeltaSet(GENERIC_DELTA_DOWN_VALUE);
  }

  if(IsLightnessChanged())
  {
    PrintLightness();

    SendLightLightnessSet(lightnessActual);
  }

  if(on)
  {
    on = false;
    SendGenericOnOffSet(GENERIC_ON_OFF_ON);
  }

  if(off)
  {
    off = false;
    SendGenericOnOffSet(GENERIC_ON_OFF_OFF);
  }
}

/*************************************************************************
 *
 * Instance Index functions
 *
 *************************************************************************/

uint8_t GetInstanceIndexBasedOnModelId(uint16_t model_id)
{
  uint8_t numberOfRegisteredModels = EEPROM.read(0);

  uint8_t j;
  for(j = 0; j < numberOfRegisteredModels; j++)
  {
    if(EEPROM.read(2*j+2) == highByte(model_id) && EEPROM.read(2*j+1) == lowByte(model_id))
    {
      return j+1;
    }
  }

  return 0;
}

void AddModelIdToInstanceIndexArray(uint16_t model_id)
{
  uint8_t numberOfRegisteredModels = EEPROM.read(0);

  EEPROM.write((numberOfRegisteredModels*2+1), lowByte(model_id));
  EEPROM.write((numberOfRegisteredModels*2+2), highByte(model_id));
  
  EEPROM.write((0), numberOfRegisteredModels+1);
  
  uint8_t instanceCheck = GetInstanceIndexBasedOnModelId(model_id);
  Serial.print("Model ");
  Serial.print(model_id, HEX);
  Serial.print(" saved as instanceIndex ");
  Serial.println(instanceCheck);
}

/*************************************************************************
 *
 * Static functions
 *
 *************************************************************************/

bool IsLightnessChanged(void)
{
  static int32_t prevLightness;
  bool isLightnessChanged = false;

  if (prevLightness != lightnessActual)
  {
    isLightnessChanged = true;
    prevLightness = lightnessActual; 
  }

  return isLightnessChanged;
}

void UARTCommandProcess(void)
{
  if (UARTCommandReceive())
  {
    switch(RxFrame.Cmd)
    {
      case UART_CMD_PING:
      {
        uint8_t buf[3] = {0x11, 0x22, 0x33};
        UARTCommandSend(3,UART_CMD_PONG,buf);                                                                          
        break;
      }
      case UART_CMD_AVAILABLE_MODELS:
      {
        if (RxFrame.Len)  
        {
          uint8_t buf[8];
          uint8_t cnt = 0;
          uint8_t instanceindex_cnt = 1;

          EEPROM.write(0x00, 0x00);
          
          for (uint8_t i=0; i < RxFrame.Len; i+=2)
          {
            if (RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT) &&              
                RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT))
            {
              buf[cnt++] = lowByte(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT);
              buf[cnt++] = highByte(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT);
              AddModelIdToInstanceIndexArray(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT);
            }

            if (RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT) &&
                RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT))
            {
              buf[cnt++] = lowByte(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT);
              buf[cnt++] = highByte(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT);
              AddModelIdToInstanceIndexArray(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT);
            }

            if (RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT) &&              
                RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT))
            {
              buf[cnt++] = lowByte(MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT);
              buf[cnt++] = highByte(MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT);
              AddModelIdToInstanceIndexArray(MESH_MODEL_ID_GENERIC_ONPOWERUP_CLIENT);
            }

            if (RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT) &&              
                RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT))
            {
              buf[cnt++] = lowByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT);
              buf[cnt++] = highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT);
              AddModelIdToInstanceIndexArray(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT);
            }

            if (RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT) &&              
                RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT))
            {
              buf[cnt++] = lowByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT);
              buf[cnt++] = highByte(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT);
              AddModelIdToInstanceIndexArray(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT);
            }
          }
          UARTCommandSend(cnt,UART_CMD_SET_INSTANCES,buf);
        }
        break;
      }
      case UART_CMD_CREATED_INSTANCES:
      {
        break;
      }
      case UART_CMD_SYNC_START:
      {
        UARTCommandSend(0,UART_CMD_SYNC_FINISH,NULL);
        break;
      }
    }
  }
}

uint8_t UARTCommandReceive(void)
{
  bool isCRCValid = false;
  static uint16_t sum;

  while (Serial2.available())
  {
    uint8_t c = Serial2.read();  
    
    switch (RxFrame.Cnt)
    {
      case 0:
        if (c == PREAMBLE_BYTE_1)
          RxFrame.Cnt++;
        else
          RxFrame.Cnt = 0;
        break;
      case 1:
        if (c == PREAMBLE_BYTE_2)
          RxFrame.Cnt++;
        else
          RxFrame.Cnt = 0;
        break;
      case 2:
        RxFrame.Len = c;
        RxFrame.Cnt++;
        break;
      case 3:
        RxFrame.Cmd = c;
        RxFrame.Cnt++;
        break;
      default:
        switch (RxFrame.Cnt - 4 - RxFrame.Len)
        {
          case 0:  
            sum = c;
            RxFrame.Cnt++;
            break;
          case 1:    
            sum += ((uint16_t)c)<<8;  
            if (sum == CalcCRC16(RxFrame.Len, RxFrame.Cmd, RxFrame.Buf))      
            {
              isCRCValid = true;            
            }
            RxFrame.Cnt = 0;
            break;
          default:
            RxFrame.Buf[RxFrame.Cnt - 4] = c;
            RxFrame.Cnt++;
            break;
        }
        break;
    }
  }

  if (isCRCValid)
    PrintDebug(0, RxFrame.Len, RxFrame.Cmd, RxFrame.Buf, sum);

  return isCRCValid;
}

void UARTCommandSend(uint8_t Len,uint8_t Cmd, uint8_t *Buf)
{ 
  uint16_t sum;
  
  Serial2.write(PREAMBLE_BYTE_1);
  Serial2.write(PREAMBLE_BYTE_2);
  Serial2.write(Len);
  Serial2.write(Cmd);
  
  for (int i = 0; i < Len; i++)   
    Serial2.write(Buf[i]);
  
  sum = CalcCRC16(Len, Cmd, Buf );
  Serial2.write(lowByte(sum));
  Serial2.write(highByte(sum));
  
  PrintDebug(1, Len, Cmd, Buf, sum);
}

/*************************************************************************
 *
 * Mesh commands transmit functions
 *
 *************************************************************************/

void SendGenericOnOffSet(bool value)
{
  uint8_t buf[6];
  uint8_t cnt = 0;
  static uint8_t tid = 0;

  buf[cnt++] = GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_GENERIC_ONOFF_CLIENT);
  buf[cnt++] = 0x00;    
  buf[cnt++] = lowByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[cnt++] = highByte(MESH_MESSAGE_GENERIC_ONOFF_SET_UNACKNOWLEDGED);
  buf[cnt++] = (value) ? 0x01 : 0x00;
  buf[cnt++] = tid++;
  UARTCommandSend(cnt, UART_CMD_MESH, buf);
}

void SendLightLightnessSet(uint16_t value)
{
  uint8_t buf[9];
  uint8_t cnt = 0;
  static uint8_t tid = 0;

  buf[cnt++] = GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_LIGHT_LIGHTNESS_CLIENT);
  buf[cnt++] = 0x00;
  buf[cnt++] = lowByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[cnt++] = highByte(MESH_MESSAGE_LIGHT_LIGHTNESS_SET_UNACKNOWLEDGED);
  buf[cnt++] = lowByte(value);
  buf[cnt++] = highByte(value);
  buf[cnt++] = tid++;
  buf[cnt++] = 0x00;
  buf[cnt++] = 0x00;
  UARTCommandSend(cnt, UART_CMD_MESH, buf);
}

void SendGenDeltaSet(int32_t value)
{
  uint8_t buf[9];
  uint8_t cnt = 0;
  static uint8_t tid = 0;

  buf[cnt++] = GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_GENERIC_LEVEL_CLIENT);
  buf[cnt++] = 0x00;
  buf[cnt++] = lowByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[cnt++] = highByte(MESH_MESSAGE_GENERIC_DELTA_SET_UNACKNOWLEDGED);
  buf[cnt++] = ((value >> 0) & 0xFF);
  buf[cnt++] = ((value >> 8) & 0xFF);
  buf[cnt++] = ((value >> 16) & 0xFF);
  buf[cnt++] = ((value >> 24) & 0xFF);
  buf[cnt++] = tid++;
  
  UARTCommandSend(cnt, UART_CMD_MESH, buf);
}

void SendLightLightnessControllerModeSet(bool value)
{
  uint8_t buf[5];
  uint8_t cnt = 0;

  buf[cnt++] = GetInstanceIndexBasedOnModelId(MESH_MODEL_ID_LIGHT_LIGHTNESS_CONTROLLER_CLIENT);
  buf[cnt++] = 0x00;    
  buf[cnt++] = lowByte(MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_SET_UNACKNOWLEDGED);          
  buf[cnt++] = highByte(MESH_MESSAGE_LIGHT_LIGHTNESS_CONTROLLER_MODE_SET_UNACKNOWLEDGED);
  buf[cnt++] = (uint8_t)value;                        
  UARTCommandSend(cnt, UART_CMD_MESH, buf); 
}

/*************************************************************************
 *
 * Hardware setup functions
 *
 *************************************************************************/

void SetupDebug(void)
{
  Serial.begin(115200);
}

void SetupUART(void)
{
  Serial2.begin(57600);
}
void SetupEncoder(void)
{
  pinMode(PIN_ENCODER_A,INPUT_PULLUP);
  pinMode(PIN_ENCODER_B,INPUT_PULLUP);
  pinMode(PIN_ENCODER_CLICK,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), InterruptEncoderAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), InterruptEncoderAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLICK), InterruptEncoderClick, FALLING);
}

void SetupPB(void)
{
  pinMode(PB_ON,INPUT_PULLUP);
  pinMode(PB_OFF,INPUT_PULLUP);  
  pinMode(PB_DELTA_UP, INPUT_PULLUP);
  pinMode(PB_DELTA_DOWN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PB_ON), InterruptOnPBClick, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_OFF), InterruptOffPBClick, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_DELTA_UP), InterruptDeltaUpPBClick, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_DELTA_DOWN), InterruptDeltaDownPBClick, FALLING);
}

/*************************************************************************
 *
 * Checksum calculation
 *
 *************************************************************************/

uint16_t CalcCRC16(byte Len, byte Cmd, byte* data) 
{
  uint16_t crc = 0xffff;
  uint8_t i;

  crc = __calcCRC(Len, crc);
  crc = __calcCRC(Cmd, crc);
  for (i = 0; i < Len; i++) 
    crc = __calcCRC(data[i], crc);  

  return crc;
}

uint16_t __calcCRC(byte data, uint16_t crc) {
  byte i;
  for (i = 0; i < 8; i++) {
    if (((crc & 0x8000) >> 8) ^ (data & 0x80)) {
      crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
      crc = (crc << 1);
    }
    data <<= 1;
  }

  return crc;
}

/*************************************************************************
 *
 * Interrupts callback functions
 *
 *************************************************************************/

void InterruptEncoderAB(void)
{
  static uint8_t pattern;

  noInterrupts();
  
  pattern <<= 1;
  pattern |=  digitalRead(PIN_ENCODER_B);
  pattern <<= 1;
  pattern |=  digitalRead(PIN_ENCODER_A);
    
  if ((pattern & ENCODER_6BIT_MASK) == ENCODER_RIGHT_TURN_PATTERN)
  {
    lightnessActual += LIGHTNESS_STEP;
  }
  else if ((pattern & ENCODER_6BIT_MASK) == ENCODER_LEFT_TURN_PATTERN)
  {
    lightnessActual -= LIGHTNESS_STEP;
  }

  if(lightnessActual < LIGHTNESS_MIN)
  {
    lightnessActual = LIGHTNESS_MIN;
  }
  if(lightnessActual > LIGHTNESS_MAX)
  {
    lightnessActual = LIGHTNESS_MAX;
  }

  if(lightnessActual > 0)
  {
    lightnessLast = lightnessActual;
  }

  interrupts();
}

void InterruptEncoderClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);

  if(digitalRead(PIN_ENCODER_CLICK)) return;

  lightLcMode = true;
}

void InterruptOnPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);

  if(digitalRead(PB_ON)) return;

  on = true;
}

void InterruptOffPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);

  if (digitalRead(PB_OFF)) return; 

  off = true;
}

void InterruptDeltaUpPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);

  if (digitalRead(PB_DELTA_UP)) return;

  deltaUp = true;
}

void InterruptDeltaDownPBClick(void)
{
  delay(BUTTON_DEBOUNCE_TIME_MS);

  if (digitalRead(PB_DELTA_DOWN)) return;

  deltaDown = true;
}

/*************************************************************************
 *
 * Debug print for console
 *
 *************************************************************************/

void PrintLightness(void)
{
  Serial.print("Current Lightness is ");
  Serial.print(lightnessActual);
  Serial.print(" (");
  Serial.print((lightnessActual * 100) / LIGHTNESS_MAX);
  Serial.println("%)");
}

void PrintMode(void)
{
  Serial.print("Light LC mode ON");
}

void PrintDebug(byte dir, byte Len, byte Cmd, byte *buf, uint16_t crc)
{
  const char*CmdName[] = {
    "Unknown         ",
    "Ping            ",
    "Pong            ",
    "AvailableModels ",
    "SetInstances    ",
    "CreatedInstances",
    "SyncStart       ",
    "Mesh            ",
    "OpcodeError     ",
    "SyncFinish      "};
  
  Serial.print(!dir ? "Received ": "    Send ");
  Serial.print(CmdName[Cmd]);

  if(Cmd <= 9)
  {
    for (int i = 0; i < Len; i++)
    {
      PrintByteHex(buf[i]);
    }
    if (dir)
    {
      PrintByteHex(lowByte(crc));
      PrintByteHex(highByte(crc));
    }
    Serial.println();
  }
}

void PrintByteHex(uint8_t data)
{
  if(data < 0x10)
  {
    Serial.print(" 0x0");
  }
  else
  {
    Serial.print(" 0x");
  }

  Serial.print(data, HEX);
}

