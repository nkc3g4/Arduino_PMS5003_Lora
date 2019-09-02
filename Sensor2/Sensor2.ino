/**
 * E32-TTL-100 Transceiver Interface
 *
 * @author Bob Chen (bob-0505@gotmail.com)
 * @date 1 November 2017
 * https://github.com/Bob0505/E32-TTL-100
 */
#include <SoftwareSerial.h>

#include "E32-TTL-100.h"

/*
 need series a 4.7k Ohm resistor between .
 UNO/NANO(5V mode)                E32-TTL-100
    *--------*                      *------*
    | D7     | <------------------> | M0   |
    | D8     | <------------------> | M1   |
    | A0     | <------------------> | AUX  |
    | D10(Rx)| <---> 4.7k Ohm <---> | Tx   |
    | D11(Tx)| <---> 4.7k Ohm <---> | Rx   |
    *--------*                      *------*
*/
#define M0_PIN  7
#define M1_PIN  8
#define AUX_PIN A0
#define SOFT_RX 10
#define SOFT_TX 11

#define COM_PIN 2  //green
#define NC_PIN 3  //yellow
#define NO_PIN A2  //blue

const char ST_SIG = 0xff;
const char ADDR = 2;
const char CHANNEL = 0x17;
const char STARTSIGN = 0xa5;
const char ENDSIGN = 0x5a;
const char NEWLINESIG = 0x0a;

SoftwareSerial softSerial(SOFT_RX, SOFT_TX);  // RX, TX
SoftwareSerial mySerial(2, 3);
static unsigned char ucRxBuffer[250];
static unsigned char ucRxCnt = 0;
long  pmcf10 = 0;
long  pmcf25 = 0;
long  pmcf100 = 0;
long  pmat10 = 0;
long  pmat25 = 0;
long  pmat100 = 0;
long  pmcount03 = 0;
long  pmcount05 = 0;
long  pmcount10 = 0;
long  pmcount25 = 0;
long  pmcount50 = 0;
long  pmcount100 = 0;

union floatData{
  float num;
  unsigned char data[4]; 
}command_data1,command_data2,command_data3,command_data4,command_data5,command_data6,command_data7,command_data8,command_data9,command_data10;   // double to bytes union =_=



//=== AUX ===========================================+
bool AUX_HL;
bool ReadAUX()
{
  int val = analogRead(AUX_PIN);

  if(val<50)
  {
    AUX_HL = LOW;
  }else {
    AUX_HL = HIGH;
  }

  return AUX_HL;
}

//return default status
RET_STATUS WaitAUX_H()
{
  RET_STATUS STATUS = RET_SUCCESS;

  uint8_t cnt = 0;
  uint8_t data_buf[100], data_len;

  while((ReadAUX()==LOW) && (cnt++<TIME_OUT_CNT))
  {
    Serial.print(".");
    delay(100);
  }

  if(cnt==0)
  {
  }
  else if(cnt>=TIME_OUT_CNT)
  {
    STATUS = RET_TIMEOUT;
    Serial.println(" TimeOut");
  }
  else
  {
    Serial.println("");
  }

  return STATUS;
}
//=== AUX ===========================================-
//=== Mode Select ===================================+
bool chkModeSame(MODE_TYPE mode)
{
  static MODE_TYPE pre_mode = MODE_INIT;

  if(pre_mode == mode)
  {
    //Serial.print("SwitchMode: (no need to switch) ");  Serial.println(mode, HEX);
    return true;
  }
  else
  {
    Serial.print("SwitchMode: from ");  Serial.print(pre_mode, HEX);  Serial.print(" to ");  Serial.println(mode, HEX);
    pre_mode = mode;
    return false;
  }
}

void SwitchMode(MODE_TYPE mode)
{
  if(!chkModeSame(mode))
  {
    WaitAUX_H();

    switch (mode)
    {
      case MODE_0_NORMAL:
        // Mode 0 | normal operation
        digitalWrite(M0_PIN, LOW);
        digitalWrite(M1_PIN, LOW);
        break;
      case MODE_1_WAKE_UP:
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M1_PIN, LOW);
        break;
      case MODE_2_POWER_SAVIN:
        digitalWrite(M0_PIN, LOW);
        digitalWrite(M1_PIN, HIGH);
        break;
      case MODE_3_SLEEP:
        // Mode 3 | Setting operation
        digitalWrite(M0_PIN, HIGH);
        digitalWrite(M1_PIN, HIGH);
        break;
      default:
        return ;
    }

    WaitAUX_H();
    delay(10);
  }
}
//=== Mode Select ===================================-
//=== Basic cmd =====================================+
void cleanUARTBuf()
{
  bool IsNull = true;

  while (softSerial.available())
  {
    IsNull = false;

    softSerial.read();
  }
}

void triple_cmd(SLEEP_MODE_CMD_TYPE Tcmd)
{
  uint8_t CMD[3] = {Tcmd, Tcmd, Tcmd};
  softSerial.write(CMD, 3);
  delay(50);  //need ti check
}

RET_STATUS Module_info(uint8_t* pReadbuf, uint8_t buf_len)
{
  RET_STATUS STATUS = RET_SUCCESS;
  uint8_t Readcnt, idx;

  Readcnt = softSerial.available();
  //Serial.print("softSerial.available(): ");  Serial.print(Readcnt);  Serial.println(" bytes.");
  if (Readcnt == buf_len)
  {
    for(idx=0;idx<buf_len;idx++)
    {
      *(pReadbuf+idx) = softSerial.read();
      Serial.print(" 0x");
      Serial.print(0xFF & *(pReadbuf+idx), HEX);    // print as an ASCII-encoded hexadecimal
    } Serial.println("");
  }
  else
  {
    STATUS = RET_DATA_SIZE_NOT_MATCH;
    Serial.print("  RET_DATA_SIZE_NOT_MATCH - Readcnt: ");  Serial.println(Readcnt);
    cleanUARTBuf();
  }

  return STATUS;
}
//=== Basic cmd =====================================-
//=== Sleep mode cmd ================================+
RET_STATUS Write_CFG_PDS(struct CFGstruct* pCFG)
{
  softSerial.write((uint8_t *)pCFG, 6);

  WaitAUX_H();
  delay(1200);  //need ti check

  return RET_SUCCESS;
}

RET_STATUS Read_CFG(struct CFGstruct* pCFG)
{
  RET_STATUS STATUS = RET_SUCCESS;

  //1. read UART buffer.
  cleanUARTBuf();

  //2. send CMD
  triple_cmd(R_CFG);

  //3. Receive configure
  STATUS = Module_info((uint8_t *)pCFG, sizeof(CFGstruct));
  if(STATUS == RET_SUCCESS)
  {
  Serial.print("  HEAD:     ");  Serial.println(pCFG->HEAD, HEX);
  Serial.print("  ADDH:     ");  Serial.println(pCFG->ADDH, HEX);
  Serial.print("  ADDL:     ");  Serial.println(pCFG->ADDL, HEX);

  Serial.print("  CHAN:     ");  Serial.println(pCFG->CHAN, HEX);
  }

  return STATUS;
}

RET_STATUS Read_module_version(struct MVerstruct* MVer)
{
  RET_STATUS STATUS = RET_SUCCESS;

  //1. read UART buffer.
  cleanUARTBuf();

  //2. send CMD
  triple_cmd(R_MODULE_VERSION);

  //3. Receive configure
  STATUS = Module_info((uint8_t *)MVer, sizeof(MVerstruct));
  if(STATUS == RET_SUCCESS)
  {
    Serial.print("  HEAD:     0x");  Serial.println(MVer->HEAD, HEX);
    Serial.print("  Model:    0x");  Serial.println(MVer->Model, HEX);
    Serial.print("  Version:  0x");  Serial.println(MVer->Version, HEX);
    Serial.print("  features: 0x");  Serial.println(MVer->features, HEX);
  }

  return RET_SUCCESS;
}

void Reset_module()
{
  triple_cmd(W_RESET_MODULE);

  WaitAUX_H();
  delay(1000);
}

RET_STATUS SleepModeCmd(uint8_t CMD, void* pBuff)
{
  RET_STATUS STATUS = RET_SUCCESS;

  Serial.print("SleepModeCmd: 0x");  Serial.println(CMD, HEX);
  WaitAUX_H();

  SwitchMode(MODE_3_SLEEP);

  switch (CMD)
  {
    case W_CFG_PWR_DWN_SAVE:
      STATUS = Write_CFG_PDS((struct CFGstruct* )pBuff);
      break;
    case R_CFG:
      STATUS = Read_CFG((struct CFGstruct* )pBuff);
      break;
    case W_CFG_PWR_DWN_LOSE:

      break;
    case R_MODULE_VERSION:
      Read_module_version((struct MVerstruct* )pBuff);
      break;
    case W_RESET_MODULE:
      Reset_module();
      break;

    default:
      return RET_INVALID_PARAM;
  }

  WaitAUX_H();
  return STATUS;
}
//=== Sleep mode cmd ================================-

RET_STATUS SettingModule(struct CFGstruct *pCFG)
{
  RET_STATUS STATUS = RET_SUCCESS;

#ifdef Device_A
  pCFG->ADDH = DEVICE_A_ADDR_H;
  pCFG->ADDL = DEVICE_A_ADDR_L;
#else
  pCFG->ADDH = DEVICE_B_ADDR_H;
  pCFG->ADDL = DEVICE_B_ADDR_L;
#endif

  pCFG->OPTION_bits.trsm_mode =TRSM_FP_MODE;
  pCFG->OPTION_bits.tsmt_pwr = TSMT_PWR_10DB;

  STATUS = SleepModeCmd(W_CFG_PWR_DWN_SAVE, (void* )pCFG);

  SleepModeCmd(W_RESET_MODULE, NULL);

  STATUS = SleepModeCmd(R_CFG, (void* )pCFG);

  return STATUS;
}

RET_STATUS ReceiveMsg(uint8_t *pdatabuf, uint8_t *data_len)
{

  RET_STATUS STATUS = RET_SUCCESS;
  uint8_t idx;

  SwitchMode(MODE_0_NORMAL);
  *data_len = softSerial.available();

  if (*data_len > 0)
  {
    Serial.print("ReceiveMsg: ");  Serial.print(*data_len);  Serial.println(" bytes.");

    for(idx=0;idx<*data_len;idx++)
      *(pdatabuf+idx) = softSerial.read();

    for(idx=0;idx<*data_len;idx++)
    {
      Serial.print(" 0x");
      Serial.print(0xFF & *(pdatabuf+idx), HEX);    // print as an ASCII-encoded hexadecimal
    } Serial.println("");
  }
  else
  {
    STATUS = RET_NOT_IMPLEMENT;
  }

  return STATUS;
}

RET_STATUS SendMsg()
{
  unsigned char SendBuf[21] = {0};
  //while(!mySerial.available()){
    //Serial.println("NotAva");
  //delay(100);
  //}
  while (mySerial.available()) {
    //Serial.println("mySerialAv");
    CopeSerialData(mySerial.read());
  }
  float smoke = pmat25;
  Serial.println(smoke);

  SendBuf[0] = 0xA5;
  SendBuf[1] = 4;//Address
  command_data1.num = 0; //light
  command_data2.num = smoke;  //temp
  command_data3.num = 0;  //Humidity
  command_data4.num = 0; //CO2
  for(int i = 0;i < 4;i++) {
    SendBuf[i + 3] = command_data1.data[i];
    SendBuf[i + 7] = command_data2.data[i];
    SendBuf[i + 11] = command_data3.data[i];
    SendBuf[i + 15] = command_data4.data[i];
  }
  SendBuf[19] = 0x0d;
  SendBuf[20] = 0x0a;
  
  RET_STATUS STATUS = RET_SUCCESS;

  SwitchMode(MODE_0_NORMAL);

  if(ReadAUX()!=HIGH)
  {
    return RET_NOT_IMPLEMENT;
  }
  delay(10);
  if(ReadAUX()!=HIGH)
  {
    return RET_NOT_IMPLEMENT;
  }


  softSerial.write(SendBuf, 21);

  return STATUS;
}

//The setup function is called once at startup of the sketch
void setup()
{
  RET_STATUS STATUS = RET_SUCCESS;
  struct CFGstruct CFG;
  struct MVerstruct MVer;

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //pinMode(COM_PIN, OUTPUT);
  //pinMode(NC_PIN, INPUT);
  //pinMode(NO_PIN, INPUT);
  //digitalWrite(COM_PIN, HIGH);

  softSerial.begin(9600);
  Serial.begin(9600);
  mySerial.begin(9600);


  //STATUS = SleepModeCmd(R_CFG, (void* )&CFG);
  //STATUS = SettingModule(&CFG);

  //STATUS = SleepModeCmd(R_MODULE_VERSION, (void* )&MVer);

  // Mode 0 | normal operation
  SwitchMode(MODE_0_NORMAL);

  //self-check initialization.
  WaitAUX_H();
  delay(10);
  
  if(STATUS == RET_SUCCESS)
    Serial.println("Setup init OK!!");
}

void blinkLED()
{
  static bool LedStatus = LOW;

  digitalWrite(LED_BUILTIN, LedStatus);
  LedStatus = !LedStatus;
}

// The loop function is called in an endless loop
void loop()
{
  //uint8_t data_buf[100], data_len;


  if(SendMsg()==RET_SUCCESS)
  {
    //Serial.println("Sent Data");
    blinkLED();
  }


  delay(random(300, 400));
}

char CopeSerialData(unsigned char ucData) {

  ucRxBuffer[ucRxCnt++] = ucData;
  if (ucRxBuffer[0] != 0x42 && ucRxBuffer[1] != 0x4D) {
    ucRxCnt = 0;
    return ucRxCnt;
  }
  if (ucRxCnt < 32) {
    return ucRxCnt;
  }
  else {
    for (int i = 0; i < 32; i++) {
      Serial.print(ucRxBuffer[i]);
      Serial.print("  ");
    }
    Serial.println("");
    pmcf10 = (float)ucRxBuffer[4] * 256 + (float)ucRxBuffer[5]; Serial.print("PM1.0_CF1:"); Serial.print(pmcf10); Serial.print("   ");
    pmcf25 = (float)ucRxBuffer[6] * 256 + (float)ucRxBuffer[7]; Serial.print("PM2.5_CF1:"); Serial.print(pmcf25); Serial.print("   ");
    pmcf100 = (float)ucRxBuffer[8] * 256 + (float)ucRxBuffer[9]; Serial.print("PM10_CF1:"); Serial.print(pmcf100); Serial.println("   ");
    pmat10 = (float)ucRxBuffer[10] * 256 + (float)ucRxBuffer[11];  Serial.print("PM1.0_AT:"); Serial.print(pmat10); Serial.print("   ");
    pmat25 = (float)ucRxBuffer[12] * 256 + (float)ucRxBuffer[13];  Serial.print("PM2.5_AT:"); Serial.print(pmat25); Serial.print("   ");
    pmat100 = (float)ucRxBuffer[14] * 256 + (float)ucRxBuffer[15];  Serial.print("PM10_AT:"); Serial.print(pmat100); Serial.println("   ");
    pmcount03 = (float)ucRxBuffer[16] * 256 + (float)ucRxBuffer[17];  Serial.print("PMcount0.3:"); Serial.print(pmcount03); Serial.print("   ");
    pmcount05 = (float)ucRxBuffer[18] * 256 + (float)ucRxBuffer[19];  Serial.print("PMcount0.5:"); Serial.print(pmcount05); Serial.print("   ");
    pmcount10 = (float)ucRxBuffer[20] * 256 + (float)ucRxBuffer[21];  Serial.print("PMcount1.0:"); Serial.print(pmcount10); Serial.println("   ");
    pmcount25 = (float)ucRxBuffer[22] * 256 + (float)ucRxBuffer[23];  Serial.print("PMcount2.5:"); Serial.print(pmcount25); Serial.print("   ");
    pmcount50 = (float)ucRxBuffer[24] * 256 + (float)ucRxBuffer[25];  Serial.print("PMcount5.0:"); Serial.print(pmcount50); Serial.print("   ");
    pmcount100 = (float)ucRxBuffer[26] * 256 + (float)ucRxBuffer[27];  Serial.print("PMcount10:"); Serial.print(pmcount100); Serial.println("   ");
    Serial.println(" *****************************************************************  "); 
    ucRxCnt = 0;
    return ucRxCnt;
  }
}
