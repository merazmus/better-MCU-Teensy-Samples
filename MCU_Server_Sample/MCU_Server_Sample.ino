/*
 * UARTModem.ino
 *
  * Created: 10.02.2017 09:17:27
  * Author : Maciej Witali�ski
 */ 

#include <TimerOne.h>
#include <math.h>

#define SERIAL_BUFFER_SIZE 0xff

#define debug  1



/*********************************************************************************************************************************
*   structure definition
**********************************************************************************************************************************/

// place to receive data from UART
struct RxFrame
{
  uint8_t     Len;                                    // lehght of data  
  uint8_t     Cmd;                                    // command code
  uint8_t     Buf[20];                                  // data
  uint8_t     Cnt;                                    // current received byte couter
} RxFrame;


// dimming process states
struct DimLight
{
  uint16_t  Target;
  uint16_t  Current;
  uint16_t  x;
  int16_t   slope;
} Light;

struct MCUver
{
  uint8_t ver_ID;
  uint8_t ver_minor;
  uint8_t ver_major;
} MCUver = {30,0,90};


/*********************************************************************************************************************************
*   UART commands definition
**********************************************************************************************************************************/

#define CMD_Ping          0x01
#define CMD_Pong          0x02
#define CMD_AvailableModels     0x03
#define CMD_SetInstances      0x04                           
#define CMD_CreatedInstances    0x05
#define CMD_SyncStart       0x06
#define CMD_Mesh          0x07
#define CMD_OpcodeError       0x08
#define CMD_SyncFinish        0x09

/*********************************************************************************************************************************
*   Mesh Opcodes
**********************************************************************************************************************************/

#define MESH_MODEL_ID_LL      0x1301                            // Light Lightness Model ID
#define MESH_MESSAGE_SET_LL     0x824C                            // LL set message
#define MESH_MESSAGE_STATUS_LL    0x824E                            // LL staus message
#define MESH_MESSAGE_GET_LL     0x824B                            // LL get message

 
/*********************************************************************************************************************************
*   GPIO definition
**********************************************************************************************************************************/
                                                
#define LED_PIN         13                                // the number of the LED pin
#define PWM_PIN         8                               // the number of the PWM pin
#define ON_SWITCH       7                               // on switch (active low state)
#define OFF_SWITCH        6                               // off switch (active low state)


/*********************************************************************************************************************************
*   timers
**********************************************************************************************************************************/

#define DIMM_INTERRUPT_TIME   5                               // dimming control interrupt interval [ms]

/*********************************************************************************************************************************
*   setup
**********************************************************************************************************************************/

void setup()
{
  Serial.begin(115200);                                     // console port
  Serial1.begin(57600);                                     // modem port
  
  pinMode(LED_PIN, OUTPUT);                                   // control LED
  pinMode(PWM_PIN, OUTPUT);                                   // light PWM pin
  pinMode(ON_SWITCH, INPUT_PULLUP);                               // switch on
  pinMode(OFF_SWITCH, INPUT_PULLUP);                              // switch off

  Timer1.initialize(DIMM_INTERRUPT_TIME*1000);                          // interrupt inerval of dimming process
  Timer1.attachInterrupt(DimmInterrupt);                            // function called to dimming process
}


/*********************************************************************************************************************************
*   main loop
**********************************************************************************************************************************/

void loop()
{     
  static uint32_t t1=millis();
  static uint32_t t2=millis();
  
  
  if (t1+20 < millis())
  {
    // 20ms interval switches polling 
    t1=millis();
    
    if (!digitalRead(ON_SWITCH))
    {
      SendToModem_SetLightCMD(0xFFFF,500);                        // send to Modem max lightness with 500ms transition time
      t1+=100;                                        // switch bouncing dead time
    }
    else
    if (!digitalRead(OFF_SWITCH))
    {
      SendToModem_SetLightCMD(0, 1000);                         // send to Modem zero lightness with 1000ms transition time
      t1+=100;                                        // switch bouncing dead time
    }   
  }
  
  
  
  //Performance test
  /*
  if (t2+10 < millis())
  {   
    t2=millis();
    CmdSend(0,CMD_Ping,NULL); 
  }
  */


  if (UARTCmdAvailable())                                   // checking received UART command
    switch(RxFrame.Cmd)
    {
      case CMD_Ping:                                    // Pong answer
              {
                uint8_t buf[3]={30,0,90};                   // ID, major ver, minor ver
                CmdSend(3,CMD_Pong,(uint8_t*)&MCUver);                            
              }                                               
              break;
      case CMD_AvailableModels:
              if (RxFrame.Len)  
              {             
                for (uint8_t i=0; i < RxFrame.Len; i+=2)
                  if ( 
                      RxFrame.Buf[i] == lowByte(MESH_MODEL_ID_LL) &&              
                      RxFrame.Buf[i+1] == highByte(MESH_MODEL_ID_LL)                      
                    )
                {
                  uint8_t buf[2];
                  buf[0]= lowByte(MESH_MODEL_ID_LL);
                  buf[1]= highByte(MESH_MODEL_ID_LL);
                  CmdSend(2,CMD_SetInstances,buf);              // create one instance for each available model                 
                  break;
                }         
              }
              break;
      case CMD_CreatedInstances:
                                                // received lists of created instances
              break;
      case CMD_SyncStart:
              {
                // prepare get command 
                uint8_t cnt=0;
                uint8_t buf[6];
                                                
                buf[cnt++]=0x01;                        // InstanceIndex
                buf[cnt++]=0x00;                        // SubIndex
                buf[cnt++]=lowByte(MESH_MESSAGE_GET_LL);            // get cmd
                buf[cnt++]=highByte(MESH_MESSAGE_GET_LL);                                         
                CmdSend(cnt,CMD_Mesh,buf);
              }
              CmdSend(0,CMD_SyncFinish,NULL);                   // synchronisation finish
                                                
              break;
      case CMD_Mesh:
              {
                uint16_t MeshCmd = RxFrame.Buf[2]+(((uint16_t)RxFrame.Buf[3])<<8);
                uint32_t t;

                switch (MeshCmd)
                {
                  case MESH_MESSAGE_STATUS_LL:                                  // received set lightness CMD                                                                   
                    ModelsCommon_ConvertFromMeshFormatToMsTransitionTime(RxFrame.Buf[8], &t);       // change transition time from mesh format to miliseconds
                    NewTargetLightness((((uint16_t)RxFrame.Buf[5])<<8) + (uint16_t)RxFrame.Buf[4] ,t);    // start dimming process
                    break;                        
                }
              }
              
    }
}


/*********************************************************************************************************************************
*   Function to send lightness to modem by UART
**********************************************************************************************************************************/

void SendToModem_SetLightCMD(uint16_t Light, uint32_t TransitionTime)
{
  uint8_t t;
  uint8_t buf[8], cnt=0;
  static uint8_t tid;

  ModelsCommon_ConvertFromMsToMeshFormatTransitionTime(TransitionTime, &t);

  buf[cnt++]=0x01;
  buf[cnt++]=0x00;    
  buf[cnt++]=lowByte(MESH_MESSAGE_SET_LL);        // set cmd
  buf[cnt++]=highByte(MESH_MESSAGE_SET_LL);
  buf[cnt++]=lowByte(Light);              // lightness
  buf[cnt++]=highByte(Light);
  buf[cnt++]=tid++; //incremented transaction id
  buf[cnt++]=t;// transition time
  buf[cnt++]=0x00; //delay
  CmdSend(cnt,CMD_Mesh,buf);
}



/*********************************************************************************************************************************
*   PWM output witch dimming curve
**********************************************************************************************************************************/

void PWMLightnesOutput(uint16_t val)
{ 
  uint32_t v;
  v= ((uint32_t)val*(uint32_t)val)>>16;                         // calculate square dimming curve                                               
  v >>=8;                                         // cast from 16 bit to 8 bit PWM resolution
  analogWrite(PWM_PIN,(uint8_t)v);                            // write port value 
}


/*********************************************************************************************************************************
*   Control of dimming process 
**********************************************************************************************************************************/

void NewTargetLightness(uint16_t Val, uint32_t TransitionTime)
{
  noInterrupts(); 

  Light.Target = Val;
  Light.x = TransitionTime / DIMM_INTERRUPT_TIME;

  if (Light.x)
    Light.slope = (int16_t)(((int32_t)Light.Target-(int32_t)Light.Current)*50l/(int32_t)Light.x);
  else
    Light.Current = Light.Target;     

  interrupts();
} 



void DimmInterrupt(void)
{
  if (Light.x)
    Light.x--;

  int32_t v = (int32_t)Light.Target -  (((int32_t)Light.x * (int32_t)Light.slope) / 50);      
  
  if (v<0)
    v=0;
  else
  if (v>0xFFFF)
    v=0xFFFF;

  Light.Current = (uint16_t)v;
  PWMLightnesOutput(Light.Current);
}




/*********************************************************************************************************************************
*   send command by UART procedure
**********************************************************************************************************************************/

void CmdSend(uint8_t Len,uint8_t Cmd, uint8_t *Buf)
{ 
  uint16_t sum;
  
  Serial1.write(0xAA);
  Serial1.write(0x55);
  Serial1.write(Len);
  Serial1.write(Cmd);
  
  for (int i = 0; i < Len; i++)   
    Serial1.write(Buf[i]);
  
  sum = CalcCRC16(Len, Cmd, Buf );
  Serial1.write((byte)(sum%256));
  Serial1.write((byte)(sum>>8));
  
  DebugPrint(1, Len, Cmd, Buf, sum);
}



/*********************************************************************************************************************************
*   receive command from UART
**********************************************************************************************************************************/

uint8_t UARTCmdAvailable(void)
{
  static uint16_t sum;
  uint8_t c, ret = 0;

  while (Serial1.available())
  {
    uint8_t c = Serial1.read();

    static uint16_t sum;    
    
    switch (RxFrame.Cnt)
    {
      case 0:
        if (c == 0xAA)
          RxFrame.Cnt++;                          // preamble - first byte ok
        else
          RxFrame.Cnt = 0;
        break;
      case 1:
        if (c == 0x55)
          RxFrame.Cnt++;                          // preamble - second byte ok
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
          case 0:                             // first byte of checksum     
            sum = c;
            RxFrame.Cnt++;
            break;
          case 1:                             // first byte of checksum     
            sum += ((uint16_t)c)<<8;  
            if (sum == CalcCRC16(RxFrame.Len, RxFrame.Cmd, RxFrame.Buf))      
            {
              digitalWrite(LED_PIN, 1 - digitalRead(LED_PIN));    // checksum is ok
              ret++;            
            }
            RxFrame.Cnt = 0;
            break;
          default:
            RxFrame.Buf[RxFrame.Cnt - 4] = c;             // save cmd data
            RxFrame.Cnt++;
            break;
        }
        break;
    }
  }

  if (ret)
    DebugPrint(0, RxFrame.Len, RxFrame.Cmd, RxFrame.Buf, sum);

  return ret;
}


/*********************************************************************************************************************************
* CheckSum calculating
**********************************************************************************************************************************/

#define crc_polynomal  0x8005

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
      crc = (crc << 1) ^ crc_polynomal;
      } else {
      crc = (crc << 1);
    }
    data <<= 1;
  }

  return crc;
}


/*********************************************************************************************************************************
* Debug print to console
**********************************************************************************************************************************/
void DebugPrint(byte dir, byte Len, byte Cmd, byte *buf, uint16_t sum)
{

  #if debug

  const char*CmdName[] = {
    "unknow           ",
    "Ping             ",
    "Pong             ",
    "AvailableModels  ",
    "SetInstances     ",
    "CreatedInstances ",
    "SyncStart        ",
    "Mesh             ",
    "OpcodeError      ",
    "SyncFinish       "};
  
  if (Cmd != CMD_Ping && Cmd != CMD_Pong)
  {

    if (Cmd > 9)
    {
      // unknow Cmd
      Cmd=0;
      Len=0;
    }
    Serial.print(!dir ? "              Received ": "Send ");
    Serial.print(CmdName[Cmd]);
    for (int i=0;i<Len;i++)
    {
      Serial.print(buf[i]);
      Serial.print(", ");
      
    }
    if (dir)
      Serial.print(sum);
    Serial.println();
  }
  #endif
}




/*********************************************************************************************************************************
* transition time convertions
**********************************************************************************************************************************/




/** Numbers of milliseconds in 100 milliseconds */
#define MODELS_COMMON_NUMBER_OF_MS_IN_100_MS    (100UL)
/** Numbers of milliseconds in 1 second */
#define MODELS_COMMON_NUMBER_OF_MS_IN_1S        (10*MODELS_COMMON_NUMBER_OF_MS_IN_100_MS)
/** Numbers of milliseconds 10 seconds */
#define MODELS_COMMON_NUMBER_OF_MS_IN_10S       (10*MODELS_COMMON_NUMBER_OF_MS_IN_1S)
/** Numbers of milliseconds 10 minutes */
#define MODELS_COMMON_NUMBER_OF_MS_IN_10MIN     (60*MODELS_COMMON_NUMBER_OF_MS_IN_10S)
#define MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_MASK            0xC0
#define MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_100_MS          0x00
#define MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_1_S             0x40
#define MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_S            0x80
#define MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_MIN          0xC0

#define MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK            0x3F
#define MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE   0x3F



bool ModelsCommon_ConvertFromMeshFormatToMsTransitionTime(uint8_t time_mesh_format, uint32_t *time_ms)
{
  /* Should be set false when transition time is unknown */
  bool transition_time_is_correct;
  
  if(MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE == (time_mesh_format & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK))
  {
    transition_time_is_correct = false;
    *time_ms = 0;
  }
  else
  {
    if(MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_MIN == (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_MASK & time_mesh_format))
    {
      *time_ms = (uint32_t)MODELS_COMMON_NUMBER_OF_MS_IN_10MIN * (time_mesh_format & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
    }
    else if(MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_S == (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_MASK & time_mesh_format))
    {
      *time_ms = (uint32_t)MODELS_COMMON_NUMBER_OF_MS_IN_10S * (time_mesh_format & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
    }
    else if(MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_1_S == (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_MASK & time_mesh_format))
    {
      *time_ms = (uint32_t)MODELS_COMMON_NUMBER_OF_MS_IN_1S * (time_mesh_format & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
    }
    else
    {
      *time_ms = (uint32_t)MODELS_COMMON_NUMBER_OF_MS_IN_100_MS * (time_mesh_format & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK);
    }

    transition_time_is_correct = true;
  }
  
  return transition_time_is_correct;
}

void ModelsCommon_ConvertFromMsToMeshFormatTransitionTime(uint32_t time_ms, uint8_t *time_mesh_format)
{
  if((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_100_MS) < MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    *time_mesh_format = (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_100_MS | ((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_100_MS) & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
  else if((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_1S) < MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    *time_mesh_format = (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_1_S | ((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_1S) & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
  else if((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_10S) < MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    *time_mesh_format = (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_S | ((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_10S) & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
  else if((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_10MIN) < MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE)
  {
    *time_mesh_format = (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | ((time_ms/MODELS_COMMON_NUMBER_OF_MS_IN_10MIN) & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
  else
  {
    *time_mesh_format = (MODELS_COMMON_TRANSITION_TIME_STEP_RESOLUTION_10_MIN | (MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_UNKNOWN_VALUE & MODELS_COMMON_TRANSITION_TIME_NUMBER_OF_STEPS_MASK));
  }
}




