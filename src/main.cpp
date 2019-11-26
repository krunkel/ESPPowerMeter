#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <asyncHTTPrequest.h>       // ESP8266HTTPClient.h  is blocking
// #include <ESP8266HTTPClient.h>   
// #include <WiFiClient.h>
#include <ArduinoJson.h>
#include <stdint.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1015_GVK.h>   // Modified library - added startContinuous_Differential

//#include <ThingSpeak.h>

#include <ConfigRogge.h>

#include <Time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//#define SER_DBG_ON  // My Debug
#define SER_DBG Serial
#define WEB_DEBUG_ON  // My Debug
#define DISPLAY_ON

const char ConfigFile[] = "/config.json";
const char BU_Config[] = "/configbu.json";
const char StateFile[] = "/state.json";
const char BU_State[] = "/statebu.json";

long LoopCount = 0;
long lastLoopCount = 0;

tm timeinfo;
//=====================================================================================================================
//===== CS5460 ======== CS5460 code inspired by https://github.com/xxzl0130/CS5460 ====================================
//=====================================================================================================================
#define SPI_SETTING     SPISettings(2000000L, MSBFIRST, SPI_MODE0) 
#define SYNC0						0xFE
#define SYNC1						0xFF

#define CS5460_REG_CONFIG		      (0x00<<1)
#define CS5460_REG_STATUS		      (0x0F<<1)
#define CS5460_REG_INTMASK        (0x1A<<1)
#define CS5460_REG_CURRENTOFFSET  (0x01<<1)
#define CS5460_REG_VOLTAGEOFFSET  (0x03<<1)
#define CS5460_REG_CYCLECOUNT     (0x05<<1)
#define CS5460_REG_CURRENT        (0x07<<1)
#define CS5460_REG_VOLTAGE        (0x08<<1)
#define CS5460_REG_POWER          (0x09<<1)
#define CS5460_REG_ENERGY   		  (0x0A<<1)
#define CS5460_REG_RMSCURRENT		  (0x0B<<1)
#define CS5460_REG_RMSVOLTAGE		  (0x0C<<1)

#define CS5460_REG_WRITE_BIT    0x40
#define CS5460_CMD_STOP         0xA0
#define CS5460_CMD_CONTCONVERT  0xE8
#define CS5460_STATUS_DRDY      0x00800000L
#define CS5460_STATUS_ERROR     0x000BF800L   // Error flags
const uint8_t CS5460_CS = 15;      // D8 op D1 Mini
const uint8_t PIN_CS5460_INT = 2;  // D4 op D1 Mini

union int32Data {
	uint32_t data32;
	uint8_t data8[4];
};
int32_t CS5460Status;
int32_t CS5460Voltage;
int32_t CS5460Current;
int32_t CS5460Power;
int32_t CS5460VoltageMn;
int32_t CS5460CurrentMn;
int32_t CS5460PowerMn = 0;
int LastCountMn = 0;
int CS5460CountMn = 0;
long CS5460CountDay = 0;  // 
const float CS5460VtDivide = 46700.00; 
const float CS5460CtDivide = 243000.00;   
const float CS5460PwDivide = 326.00;   // 332 > 316 > 321 > 323 > 320 > 325 > 326
const float CS5460CtOffset = 0.00;   // was 200

//=====================================================================================================================
//===== IP/WEB/JSON ===================================================================================================
//=====================================================================================================================
asyncHTTPrequest NRM_http_req;
asyncHTTPrequest NRD_http_req;
asyncHTTPrequest NRE_http_req;
asyncHTTPrequest PVO_http_req;
// HTTPClient http;
DynamicJsonDocument jsonDoc(2048);
AsyncWebServer server(80);

const char* ssid = MYSID;       // defined in ConfigRogge.h
const char* password = MYPW;    // defined in ConfigRogge.h
const char* hostName = "esppower";

unsigned long last5000; 
unsigned long last1000; 
unsigned long lastEvent = 0;
byte numEvents = 0;
byte lastMinute = 0;
byte lastHour = 0;
byte lastDay = 9;
byte MinuteCount = 0;
byte lastPVOutput = 99;
bool FirstNTP = true; 
bool Q_SaveStat = false; 

float HouseVoltage;
float HouseCurrent;
float HousePower;
float kWhOffset;   // from ConfigFile
float kWhOffset1=0;   // from ConfigFile
float kWhOffset2=0;   // from ConfigFile
long HouseECount;  // Incremented each minute - to SPIFFS each 5 min. - Unit: cWh
long ADS0ECount;  // Incremented each minute - to SPIFFS each 5 min. - Unit: cWh
long ADS1ECount;  // Incremented each minute - to SPIFFS each 5 min. - Unit: cWh
long HouseECountDay = 0;  // Incremented each minute - Unit: mWh
long ADS0ECountDay = 0;  // Incremented each minute - Unit: mWh
long ADS1ECountDay = 0;  // Incremented each minute - Unit: mWh
// range = -99.999 > 99.999 
//       float  has precision of +/- 6 digits - so limited to 100 Wh 
//       long  = +/-2147483647 in cWh = (100/Wh) > +/-21474.83647 kWH  
//float PowerSol1;
//float PowerSol2;
float HouseVoltageMn;
float HouseCurrentMn;
float HousePowerMn;
float PowerSol1Mn = 0;
float PowerSol2Mn = 0;
//=====================================================================================================================
//===== ADS Stuff =====================================================================================================
//=====================================================================================================================
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

byte ADSCount = 0;
byte ADSChannel = 0; 
long ADSTimer = 103100;  // timer - wordt later bepaald door CycleAvg
int16_t ADSValue[32];    // Actual measurement
uint64_t ADSClock[32];   // Micros on measurement (to determine powerfactor)
uint64_t ADSLastZC[32];  // Zerocross at time of measurement  

float ADS_Current[2] = {0,0};
float ADS_Appr_Power[2] = {0,0};
float ADS_Real_Power[2] = {0,0};
float ADS_Cur_Divide[2] = {3000,3050};// For RMS value (=) Sqrt(32) * ADS factor * 
// Used Current Transformer = 1/2000
// Range: 0-20A - 20 A = max 28,28A PP = 
// Burden = 84 Ohm > max 1,19V PP    1 A = 0,042V (20)
// ADS Gain 2 = max 2V PP - Birresolution = 0.0625mV
// 1A = 42mv/ = 672 
// float ADS_Cur_Offset[2] = {0,0}; 
int ADS_PowerCountMn[2] = {0,0}; // Normally 30 measurements / minute
double ADS_CosPhi[32][2];            // buffers for running average
byte ADS_CosPhiPnt[2];            // Pointer to next place in buffer for running average
int ADS_Shift[2]; // for callibration
double ADS_CosPhiSum[2] = {0,0};
double ADS_CosPhiAvg[2] = {1,1};
//int ADS_CosPhiDelta[2];
bool ADS_CosPhiAvgOK[2] = {false,false};
const int ADS_ZCCorrection[2] = {1150,1100};   // compensation for CT & ZeroCross measurement shift  
uint64_t ADS_ZeroCross[2] = {0,0};             // Calculated after 32 measurements
uint64_t ADS_LastZeroCross[2] = {0,0};
uint64_t ADS_MeanZeroCross[2] = {0,0};
//String LDebugADSVal[2];
//String LDebugADSMcr[2];

//const float PowerSol2Divide = 3050; //510.00 noot: incl SQRT(32); 
//float PowerSol1Offset = 0; // 160;
//const float PowerSol2Offset = 0;

//=====================================================================================================================
//===== ZeroCross Stuff ===============================================================================================
//=====================================================================================================================
const uint8_t PIN_ZEROCR_INT = 3; // RX on D1mini
volatile uint64_t TZeroCross=0; 
uint64_t LastZeroCross=0;
uint64_t MeanZeroCross=0;
uint64_t LastMeanZeroCross=0;
unsigned long LastCycle=0;
unsigned long Cycle[64]; 
unsigned long CycleSum;
unsigned long CycleAvg;
float Frequency; 
byte CyclePnt = 0;

// HouseVoltage = float(CS5460Voltage)/CS5460VtDivide;
// HouseCurrent = float(CS5460Current-CS5460CtOffset)/CS5460CtDivide;
// HousePower = float(CS5460Power)/CS5460PwDivide;
 
// ################### ThingSpeak ###################
// Free: 8K / dag - 5 per minuut
// Start: 1 / minuut
// Field 1 NetVoltage
// Field 2 CurrentHome
// Field 3 PowerHome
// Field 4 PowerSol1
// Field 5 PowerSol2
// Field 6 PowerCar
// Field 7 DayECount
// Field 8 
// unsigned long MnChannelNumber = 860156;
// const char * MnWriteAPIKey = "xxx";
// unsigned long HrChannelNumber = 863999;
// const char * HrWriteAPIKey = "xxx";

// ################### PVOutput ###################
const char * PVO_Url = "http://pvoutput.org/service/r2/addstatus.jsp";
const char * PVO_Apikey = MYPVOAPI;     // defined in ConfigRogge.h
const char * PVO_SystemId = MYPVOSYS1;  // defined in ConfigRogge.h
const char * PVO_ContType = "application/x-www-form-urlencoded";

WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP);
NTPClient timeClient(ntpUDP,"be.pool.ntp.org",3600);   //Zomertijd: NTPClient timeClient(ntpUDP,"be.pool.ntp.org",7200);
//WiFiClient NR_client;  - changed to asyncHTTPrequest
//WiFiClient PV_client;

IPAddress staticIP(192, 168, 1, 50); //ESP static ip
IPAddress gateway(192, 168, 1, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(8, 8, 8, 8);  //DNS


volatile byte IRReadADS = false; 
volatile byte IRReadCS5460 = false; 
unsigned long LastCS5460=0;
uint16_t NRErrors = 0;

//#######################################################################################################################################
//##### Interruptroutines ###############################################################################################################
//#######################################################################################################################################
//##### No processing in IR routines ####################################################################################################
//#######################################################################################################################################

void ICACHE_RAM_ATTR ReadADSTimer(void) { 
  IRReadADS = true;
}

void ICACHE_RAM_ATTR CS5460Ready(void) { 
  IRReadCS5460 = true;
}

void ICACHE_RAM_ATTR ZeroCross(void) { 
  TZeroCross = micros64(); 
}

//#####################################################################################################################
//##### ADS1115 stuff #################################################################################################
//#####################################################################################################################

//=====================================================================================================================
//===== ADSStart ======================================================================================================
//=====================================================================================================================
//===== To get 32 samples of one cycle on the slow ADS1115 we measure once each cycles + 1/32 of the cycle ============
//===== so after 33 cycles we can calculate RMS with these 32 samples =================================================
//=====================================================================================================================
void ADSStart() {
  ADSCount = 36;                   // 4 eerste metingen worden niet gebruikt - na switch ingang
  ADSChannel = ADSChannel ^ 0x01;  // afwisselend kanaal 0 en 1 
  ADSTimer = CycleAvg * 5.15625;   // CycleAvg = current cycle in us // Cycle * (1+1/32) * 5 
  ads.startContinuous_Differential(ADSChannel);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);  // Start 32 metingen () - TIM_DIV16 = 5MHz (5 ticks/us - 1677721.4 us max)
  timer1_write(ADSTimer);  // 20,625ms   53125 DIV16  // normaal 103125 
}
//=====================================================================================================================
//===== ADSProcess ====================================================================================================
//=====================================================================================================================
void ADSProcess() {   // Process 32 measurements - triggered by ReadADSTimer interrupt
  ADSCount--;
  if (ADSCount < 32) {// only last 32 sample moments
    //===== Get measurement & clock in micros =========================================================================
    ADSClock[ADSCount] = micros64();
    ADSValue[ADSCount] = ads.getLastConversionResults();
    ADSLastZC[ADSCount] = MeanZeroCross; // Remember voltage ZeroCross on sample moment for powerfactor calculation
    if (ADSCount == 0) {     // Last measurement passed;
      timer1_disable();
      yield();
      //===== Determine ZeroCross =====================================================================================
      byte cnt = 32;
      byte cntZC = 0;
      byte NumCrossDetected = 0; 
      uint64_t MyZeroCross = 0;
      //ADSOffset[ADSChannel] += ADSValue[0];
      float Mean1 = (0L+ADSValue[0]+ADSValue[31]+ADSValue[30])/3; // to avoid multiple zerocrosses we use avg over 3 measurements
      float Mean2; 
      while (cnt > 1) {
        cnt--;        
        Mean2 = (0L+ADSValue[cnt]+ADSValue[cnt-1]+ADSValue[(cnt-2)&0x1F])/3; 
        if ((Mean1 < 0) && (Mean2 >= 0)) {   // ZeroCross detected => Interpolate 2 Times
          NumCrossDetected++; 
          long MyDelta =  (ADSClock[cnt-1] - CycleAvg - ADSClock[cnt]) / (Mean2 - Mean1) * Mean1;   
          MyZeroCross = ADSClock[cnt] - MyDelta - ADS_ZCCorrection[ADSChannel];
          cntZC = cnt;
        }
        Mean1 = Mean2;
      }
      yield();
      //===== Process ZeroCross  (calculate shift & calculate running average over 32 measurements) ===================
      if (NumCrossDetected == 1) {
        ADS_LastZeroCross[ADSChannel] = ADS_ZeroCross[ADSChannel];  
        ADS_ZeroCross[ADSChannel] = MyZeroCross;
        int ThisShift = (MyZeroCross-ADSLastZC[cntZC]+CycleAvg)%CycleAvg;  // ADSLastZC[cntZC] = MeanZeroCross at zerocros - Attention Mod from negative not ok so, extra CycleAvg  ???
        ADS_Shift[ADSChannel] = ThisShift;
        double ThisCos = cos(ThisShift * TWO_PI / CycleAvg);
        //=== calculate running average ===
        ADS_CosPhiSum[ADSChannel]-=ADS_CosPhi[ADS_CosPhiPnt[ADSChannel]][ADSChannel];
        ADS_CosPhi[ADS_CosPhiPnt[ADSChannel]][ADSChannel] = ThisCos;
        ADS_CosPhiSum[ADSChannel]+=ThisCos;
        ++ADS_CosPhiPnt[ADSChannel]&=0x1F;
        if (ADS_CosPhiPnt[ADSChannel] == 0) ADS_CosPhiAvgOK[ADSChannel] = true;  // Na 32 metingen AVG = ok
        if (ADS_CosPhiAvgOK[ADSChannel]) ADS_CosPhiAvg[ADSChannel] = ADS_CosPhiSum[ADSChannel]/32;  // if buffer is full, always devide by 32
        else ADS_CosPhiAvg[ADSChannel] = ADS_CosPhiSum[ADSChannel] / ADS_CosPhiPnt[ADSChannel];     // if not divide by bufferitems at startup
        //=== end running average =========
      }
      yield();
    }
  }
  IRReadADS = false;
}
//#####################################################################################################################
//##### Config/State stuff ############################################################################################
//#####################################################################################################################
void ReadConfigFromSpiffs() {
  File file = SPIFFS.open(ConfigFile,"r");
  if (!file) {
    // Serial.println("file open failed");
    file = SPIFFS.open(BU_Config,"r");
  } 
  if (file) {
    jsonDoc.clear();
    DeserializationError error = deserializeJson(jsonDoc, file);
    if (!error) {
      kWhOffset = jsonDoc["kWhOffset"];
      kWhOffset1 = jsonDoc["kWhOffset1"];
      kWhOffset2 = jsonDoc["kWhOffset2"];
    } else {
      kWhOffset = 46108.38;
      kWhOffset1 = 4867.27;  // 4866.96 op 27/10/2019
      kWhOffset2 = 1146.33;  // =2792,475-1654,45 op 27/10/2019
    }
  } else {
    kWhOffset = 46108.38;
    kWhOffset1 = 4867.27;  // 4866.96 op 27/10/2019
    kWhOffset2 = 1146.33;  // =2792,475-1654,45 op 27/10/2019
  }
  file.close();
}

void ReadStateFromSpiffs() {
  File file = SPIFFS.open(StateFile,"r");
  if (!file) {
    // Serial.println("file open failed");
    file = SPIFFS.open(BU_State,"r");
  } 
  if (file) {
    jsonDoc.clear();
    DeserializationError error = deserializeJson(jsonDoc, file);
    if (!error) {
      HouseECount = jsonDoc["HouseECount"];
      ADS0ECount = jsonDoc["ADS0ECount"];
      ADS1ECount = jsonDoc["ADS1ECount"];
      HouseECountDay =  jsonDoc["ECntDay"];
      ADS0ECountDay = jsonDoc["ECntDay0"];
      ADS1ECountDay = jsonDoc["ECntDay1"];
      CS5460CountDay = jsonDoc["ECnt"];
    } else {
      ADS1ECount = ADS0ECount = HouseECount = 0;
    }
  } else {
    ADS1ECount = ADS0ECount = HouseECount = 0;
  }
  file.close();
}
//#####################################################################################################################
//##### TODO RTC + AT24C32 (1 miljoen cycles) # = 9 jaar ##############################################################
//#####################################################################################################################
void WriteStateToSpiffs() { // const char *filename, const Config &config) {
  SPIFFS.remove(BU_State);
  SPIFFS.rename(StateFile,BU_State);
  File file = SPIFFS.open(StateFile,"w");
  if (file) {
    jsonDoc.clear();
    jsonDoc["HouseECount"] = HouseECount;
    jsonDoc["ADS0ECount"] = ADS0ECount;
    jsonDoc["ADS1ECount"] = ADS1ECount;
    jsonDoc["ECntDay"] = HouseECountDay;
    jsonDoc["ECntDay0"] = ADS0ECountDay;
    jsonDoc["ECntDay1"] = ADS1ECountDay;
    jsonDoc["ECnt"] = CS5460CountDay;
    serializeJson(jsonDoc, file);
  }
  file.close();
}

//#####################################################################################################################
//##### Node-Red stuf #################################################################################################
//#####################################################################################################################

//=====================================================================================================================
//===== Send measurement to Node-Red server ===========================================================================
//=====================================================================================================================
void send2nodered() {
  String output;
  jsonDoc.clear();
  jsonDoc["voltage"] = (float) HouseVoltageMn;  
  jsonDoc["total_current"] = (float) HouseCurrentMn;  
  jsonDoc["total_power"] = (float) HousePowerMn;  
  jsonDoc["power_sol1"] = (float) PowerSol1Mn;
  jsonDoc["power_sol2"] = (float) PowerSol2Mn;
  jsonDoc["current_sol1"] = (float) ADS_Current[0];
  jsonDoc["current_sol2"] = (float) ADS_Current[1];
  jsonDoc["pf_sol1"] = (float) ADS_CosPhiAvg[0] * 100;
  jsonDoc["pf_sol2"] = (float) ADS_CosPhiAvg[1] * 100;
  jsonDoc["frequency"] = (float) Frequency;
  serializeJson(jsonDoc, output);
  if(NRM_http_req.readyState() == 0 || NRM_http_req.readyState() == 4) {
      NRM_http_req.open("POST", "http://192.168.1.100:1880/housepower");
      NRM_http_req.setReqHeader("Content-Type", "application/json");
      NRM_http_req.send(output);
  }
}

//=====================================================================================================================
//===== Send DayStat to Node-Red server ===============================================================================
//=====================================================================================================================
void send2noderedDay() {
  String output;
  jsonDoc.clear();
  float WhCount = HouseECountDay / 1000.00;
  float WhADS0 = ADS0ECountDay / 1000.00;
  float WhADS1 = ADS1ECountDay / 1000.00;
  jsonDoc["time"] = timeClient.getEpochTime() - 43200; // Registreer op middag vorige dag
  jsonDoc["total_power"] = (float) WhCount;  
  jsonDoc["power_sol1"] = (float) WhADS0;
  jsonDoc["power_sol2"] = (float) WhADS1;
  serializeJson(jsonDoc, output);
  if(NRD_http_req.readyState() == 0 || NRD_http_req.readyState() == 4) {
      NRD_http_req.open("POST", "http://192.168.1.100:1880/housepower_day");
      NRD_http_req.setReqHeader("Content-Type", "application/json");
      NRD_http_req.send(output);
  }
}
//=====================================================================================================================
//===== Send Event to Node-Red server =================================================================================
//=====================================================================================================================
void SendEvent2nodered(String EventString) {
	if(numEvents<4) {   // Max 4 events per sec
		String output;
		jsonDoc.clear();
		jsonDoc["node"] = "esppower"; 
		jsonDoc["event"] = EventString; 
    serializeJson(jsonDoc, output);
    if(NRE_http_req.readyState() == 0 || NRE_http_req.readyState() == 4) {
        NRE_http_req.open("POST", "http://192.168.1.100:1880/event");
        NRE_http_req.setReqHeader("Content-Type", "application/json");
        NRE_http_req.send(output);
    }
    lastEvent = millis();
    numEvents++;
	}
}

//=====================================================================================================================
//===== Send Solar Data to pvoutput.org ===============================================================================
//=====================================================================================================================
void SendData2PVOutput() {
  time_t now = timeClient.getEpochTime();
  char cTime[8]; 
  char cDate[10]; 
  localtime_r(&now, &timeinfo);
  strftime(cTime, sizeof(cTime), "%H:%M", &timeinfo);
  strftime(cDate, sizeof(cDate), "%Y%m%d", &timeinfo);
  if ((ADS0ECountDay > 5000) && (PowerSol1Mn > 10)) {
    float WhADS0 = ADS0ECountDay / 1000.00;
    if(PVO_http_req.readyState() == 0 || PVO_http_req.readyState() == 4) {
        PVO_http_req.open("POST", PVO_Url);
        PVO_http_req.setReqHeader("X-Pvoutput-Apikey", PVO_Apikey);
        PVO_http_req.setReqHeader("X-Pvoutput-SystemId", PVO_SystemId);
        PVO_http_req.setReqHeader("Content-Type", PVO_ContType);
        String postMsg = String("d=") + String(cDate);
        postMsg += String("&t=") + String(cTime);
        postMsg += String("&v1=") + String(WhADS0,2);
        postMsg += String("&v2=") + String(PowerSol1Mn,2);
        PVO_http_req.send(postMsg);
    }
  }
  /*
  if (ADS1ECountDay > 0) {
    float WhADS1 = ADS1ECountDay / 1000.00;
    http.begin(PV_client,PVO_Url);
	  http.addHeader("X-Pvoutput-Apikey", PVO_Apikey);
	  http.addHeader("X-Pvoutput-SystemId", PVO_SystemId);
	  http.addHeader("Content-Type", PVO_ContType);
   	String postMsg = String("d=") + String(cDate);
  	postMsg += String("&t=") + String(cTime);
  	postMsg += String("&v1=") + String(WhADS1,2);
  	postMsg += String("&v2=") + String(PowerSol2Mn,2);
		http.POST(output);  // int httpCode = 
		http.end();
  }
  */
  //lastPVOutput = lastMinute;
}

//#####################################################################################################################
//##### Async Webclient stuf ##########################################################################################
//#####################################################################################################################
void PVOreqResponse(void* optParm, asyncHTTPrequest* PVO_http_req, int readyState) {
  if(readyState == 4) {
    if (PVO_http_req->responseHTTPcode() != 200) {
      String ReturnMsg = String("Returncode pvoutput:") + String(PVO_http_req->responseHTTPcode());
      ReturnMsg += String(" ") + PVO_http_req->responseText();
      SendEvent2nodered(ReturnMsg);
    }
  }
}
void NRMreqResponse(void* optParm, asyncHTTPrequest* NRM_http_req, int readyState) {
  if(readyState == 4) {
    if (NRM_http_req->responseHTTPcode() != 200) {
      NRErrors++;  
      //String ReturnMsg = String("Returncode NR Min:") + String(NRM_http_req->responseHTTPcode());
      //ReturnMsg += String(" ") + NRM_http_req->responseText();
      //SendEvent2nodered(ReturnMsg);
    }
  }
}
void NRDreqResponse(void* optParm, asyncHTTPrequest* NRD_http_req, int readyState) {
  if(readyState == 4) {
    if (NRD_http_req->responseHTTPcode() != 200) {
      NRErrors++;  
    }
  }
}
void NREreqResponse(void* optParm, asyncHTTPrequest* NRE_http_req, int readyState) {
  if(readyState == 4) {
    if (NRE_http_req->responseHTTPcode() != 200) {
      NRErrors++;  
    }
  }
}

//#####################################################################################################################
//##### CS5460 stuff ##################################################################################################
//#####################################################################################################################
void CS5460_spi_start() {
  digitalWrite(CS5460_CS, LOW);
	SPI.beginTransaction(SPI_SETTING);
}
//=====================================================================================================================
void CS5460_spi_stop() {
	SPI.endTransaction();
  digitalWrite(CS5460_CS, HIGH);
}
//=====================================================================================================================
void CS5460_sendcmd(uint8_t cmd) {
  SPI.transfer(cmd);
}
//=====================================================================================================================
uint32_t CS5460_readReg(uint8_t reg) {
	uint32_t data = 0;
	SPI.transfer(reg);
	for(uint8_t i = 0;i < 3;++i) 	{
		data <<= 8;
		data |= SPI.transfer(SYNC0);
	}
	return data;
}
//=====================================================================================================================
void CS5460_writeReg(uint8_t reg, uint32_t val) {
	int32Data data;
  reg |= CS5460_REG_WRITE_BIT;
	SPI.transfer(reg);
	data.data32 = val;
	for(int8_t i = 2;i >=0; --i) 	{// CS5430 =  24bit registers > 3 LSB bytes 
		SPI.transfer(data.data8[i]);
	}
}
//=====================================================================================================================
void CS5460_clearStatus(void) {
  CS5460_writeReg(CS5460_REG_STATUS,0x00FFFFFFL);
}
//=====================================================================================================================
void CS5460_init() {
  pinMode(CS5460_CS, OUTPUT); 
	SPI.pins(14,12,13,CS5460_CS);
	SPI.begin();
	SPI.setFrequency(2000000);
  CS5460_spi_start();
	SPI.transfer(SYNC1);
	SPI.transfer(SYNC1);
	SPI.transfer(SYNC1);
	SPI.transfer(SYNC0);
  // TODO Afzonderlijk proces Calibration  -- Eenmalig
  // CS5460_sendcmd(0xDD);  //AC CALIB
  // CS5460_sendcmd(0xD9);  //DC CALIB
  delay(1000);
  CS5460_writeReg(CS5460_REG_CONFIG,0x00000061L);         // Divider = 1 + highpass filters op = 60 
  CS5460_writeReg(CS5460_REG_CURRENTOFFSET,0x00FFFF60L);   
  CS5460_writeReg(CS5460_REG_VOLTAGEOFFSET,0x00000650L);
  // CS5460_writeReg(CS5460_REG_CYCLECOUNT,0x00004E20L);  // Default 4000 > elke seconde 
  CS5460_writeReg(CS5460_REG_INTMASK,CS5460_STATUS_DRDY);
  CS5460_writeReg(CS5460_REG_STATUS,0x00FFFFFFL);         // Reset all status bits
  CS5460_sendcmd(CS5460_CMD_CONTCONVERT);                 // Start Continuous Mode
  CS5460_spi_stop();
  #ifdef SER_DBG_ON
    SER_DBG.println("init CS5460 ok");
  #endif
}
//=====================================================================================================================
void CS5460_re_init() {
  CS5460_spi_start();
  CS5460_sendcmd(CS5460_CMD_STOP);
  delay(500);
  CS5460_writeReg(CS5460_REG_CONFIG,0x00000061L);         // Divider = 1 + highpass filters op = 60 
  CS5460_writeReg(CS5460_REG_CURRENTOFFSET,0x00FFFF60L);   
  CS5460_writeReg(CS5460_REG_VOLTAGEOFFSET,0x00000650L);
  delay(500);
  CS5460_writeReg(CS5460_REG_INTMASK,CS5460_STATUS_DRDY);
  CS5460_writeReg(CS5460_REG_STATUS,0x00FFFFFFL);
  CS5460_sendcmd(CS5460_CMD_CONTCONVERT);
  CS5460_spi_stop();
  LastCS5460 = millis();
  #ifdef SER_DBG_ON
    SER_DBG.println("re_init CS5460 ok");
  #endif
}

//=====================================================================================================================
//=== Process CS5460 measurement if measurement completed (Interrupt) =================================================
//=====================================================================================================================
void CS5460Process() {
  //detachInterrupt(digitalPinToInterrupt(2));
  CS5460_spi_start();
  CS5460Power = CS5460_readReg(CS5460_REG_ENERGY);  //powerMeter.getPower();
  CS5460Current = CS5460_readReg(CS5460_REG_RMSCURRENT);  //powerMeter.getCurrent();
  CS5460Voltage = CS5460_readReg(CS5460_REG_RMSVOLTAGE);  //powerMeter.getVoltage();
  CS5460Status = CS5460_readReg(CS5460_REG_STATUS);  //powerMeter.getStatus();
  // Clear DRDY bit status
  //CS5460_writeReg(CS5460_REG_STATUS,CS5460_STATUS_DRDY);
  CS5460_writeReg(CS5460_REG_STATUS,0x00FFFFFFL);
  CS5460_spi_stop();
  if (CS5460Power & 0x00800000L ) {
    CS5460Power = (CS5460Power ^ 0xFF000000L);  // signbit from byte 3 to byte 4
    // TODO SIGN CURRENT
  } 
  CS5460PowerMn += CS5460Power;
  CS5460CurrentMn += CS5460Current;
  CS5460VoltageMn += CS5460Voltage;
  CS5460CountMn++;
  IRReadCS5460 = false;
  LastCS5460 = millis();
  //attachInterrupt(digitalPinToInterrupt(2), CS5460Ready, FALLING); // Intterrupt GPIO2 (D4 on Wemos D1)  CHANGE | RISING 
}

//=====================================================================================================================
//=== Normally CS5460 measurement each sec === restart if to long (Math error,....) ===================================
//=====================================================================================================================
void CS5460_CheckState() {
  CS5460_spi_start();
  CS5460Status = CS5460_readReg(CS5460_REG_STATUS);  
  CS5460_spi_stop();
  if ((CS5460Status & CS5460_STATUS_ERROR) > 0) {
    // Error op CS5460 => 
    SendEvent2nodered("CS5460 Restart Stat=" + String(CS5460Status));
    CS5460_re_init();
  }
  if ((CS5460Status & CS5460_STATUS_DRDY) > 0) {
    // CS5460 meting klaar - interrupt gemist... lees in volgende loop
    IRReadCS5460 = true;
  } 
}

//#####################################################################################################################
//##### EINDE CS5460 stuff ############################################################################################
//#####################################################################################################################


//#####################################################################################################################
//##### Web stuff #####################################################################################################
//#####################################################################################################################
void WebSetup() {
  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });
  server.on("/savestat", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain","Ok!" );
    Q_SaveStat = true;
  });

  //===================================================================================================================
  //server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
  //  request->send(200, "text/plain", LDebugADSVal[0] + "\n" + LDebugADSMcr[0] +"\n"+ LDebugADSVal[1] + "\n" + LDebugADSMcr[1] +"\n");
  //  //request->send(200, "text/plain", LDebugADSVal[0] + "\n" + LDebugADSVal[1] + "\n");
  //});
  //===================================================================================================================
  server.on("/status.json", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    jsonDoc.clear();
    jsonDoc["CSStat"] = CS5460Status;
    jsonDoc["Delta"] = (int)(LastMeanZeroCross - LastZeroCross);
    jsonDoc["Shift1"] = ADS_Shift[0];
    jsonDoc["Shift2"] = ADS_Shift[1];
    //jsonDoc["Shift1"] = ADS_CosPhiAvg[0];
    //jsonDoc["Shift2"] = ADS_CosPhiAvg[1];
    //jsonDoc["Delta1"] = ADS_CosPhiDelta[0];
    //jsonDoc["Delta2"] = ADS_CosPhiDelta[1];
    jsonDoc["PF1"] = ADS_CosPhiAvg[0];
    jsonDoc["PF2"] = ADS_CosPhiAvg[1];
    jsonDoc["AvgCycle"] = CycleAvg;
    jsonDoc["CS5460CountMn"] = LastCountMn;
    jsonDoc["StateMin"] = MinuteCount;
    jsonDoc["NRErrors"] = NRErrors;
    jsonDoc["loopcount"] = lastLoopCount;
    serializeJsonPretty(jsonDoc, *response);
    request->send(response);
  });
  //===================================================================================================================
  server.on("/keyvalues.json", HTTP_GET, [](AsyncWebServerRequest *request){
    HouseVoltage = float(CS5460Voltage)/CS5460VtDivide;
    HouseCurrent = float(CS5460Current-CS5460CtOffset)/CS5460CtDivide;
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    jsonDoc.clear();
    // jsonDoc["CSStat"] = CS5460Status;
    jsonDoc["Vtot"] = HouseVoltage;  
    jsonDoc["Itot"] = HouseCurrent;
    jsonDoc["PtotMn"] = HousePowerMn;
    jsonDoc["PsolMn"] = PowerSol1Mn+PowerSol2Mn;
    serializeJson(jsonDoc, *response);
    request->send(response);
  });
  //===================================================================================================================
  server.on("/values.json", HTTP_GET, [](AsyncWebServerRequest *request){
    HouseVoltage = float(CS5460Voltage)/CS5460VtDivide;
    HouseCurrent = float(CS5460Current-CS5460CtOffset)/CS5460CtDivide;
    HousePower = float(CS5460Power)/CS5460PwDivide;
    float kWhCount = kWhOffset + (HouseECount / 100000.00);
    float kWhADS0 = kWhOffset1 + (ADS0ECount / 100000.00);
    float kWhADS1 = kWhOffset2 + (ADS1ECount / 100000.00);
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    jsonDoc.clear();
    // jsonDoc["CSStat"] = CS5460Status;
    jsonDoc["Vtot"] = HouseVoltage;  
    jsonDoc["Itot"] = HouseCurrent;
    jsonDoc["Ptot"] = HousePower;
    jsonDoc["kWhCount"] = kWhCount;
    jsonDoc["kWhADS0"] = kWhADS0;
    jsonDoc["kWhADS1"] = kWhADS1;
    jsonDoc["PtotMn"] = HousePowerMn;
    jsonDoc["CurSol1"] = ADS_Current[0];
    jsonDoc["CurSol2"] = ADS_Current[1];
    jsonDoc["PowerSol1"] = ADS_Real_Power[0]/ADS_PowerCountMn[0];
    jsonDoc["PowerSol2"] = ADS_Real_Power[1]/ADS_PowerCountMn[1];
    jsonDoc["PowerFac1"] = ADS_CosPhiAvg[0];
    jsonDoc["PowerFac2"] = ADS_CosPhiAvg[1];;
    jsonDoc["PowerSol1Mn"] = PowerSol1Mn;
    jsonDoc["PowerSol2Mn"] = PowerSol2Mn;
    jsonDoc["Frequency"] = Frequency;
    serializeJsonPretty(jsonDoc, *response);
    request->send(response);
  });
  //===================================================================================================================
  server.on("/dayval.json", HTTP_GET, [](AsyncWebServerRequest *request){
    float WhCount = HouseECountDay / 1000.00;
    float WhADS0 = ADS0ECountDay / 1000.00;
    float WhADS1 = ADS1ECountDay / 1000.00;
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    jsonDoc.clear();
    jsonDoc["CSCount"] = CS5460CountDay;  
    jsonDoc["total_power"] = WhCount;
    jsonDoc["power_sol1"] = WhADS0;
    jsonDoc["power_sol2"] = WhADS1;
    serializeJsonPretty(jsonDoc, *response);
    request->send(response);
  });
  //===================================================================================================================
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  server.begin();
}

//#####################################################################################################################
//##### SETUP #########################################################################################################
//#####################################################################################################################
void setup(){
  for(byte i = 0;i < 32;i++) 	{
    Cycle[i]=20000;
    CycleSum+=Cycle[i];
    ADS_CosPhi[i][0]=0;
    ADS_CosPhi[i][1]=0;
  }
  for(byte i = 32;i < 64;i++) 	{
    Cycle[i]=20000;
    CycleSum+=Cycle[i];
  }
  CycleAvg = 20000;
  
  //#ifdef SER_DBG_ON
    SER_DBG.begin(115200);
  //#endif
  SER_DBG.setDebugOutput(false);
  //===================================================================================================================
  //=== Init WiFI =====================================================================================================
  //===================================================================================================================
  //WiFi.hostname(hostName);      // DHCP Hostname (useful for finding device for static lease)
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  //WiFi.config(staticIP, subnet, gateway, dns,0);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    #ifdef SER_DBG_ON
      SER_DBG.print(".");
    #endif
  }
  #ifdef SER_DBG_ON
    SER_DBG.println(WiFi.localIP());
  #endif
  MDNS.begin(hostName);
  //===================================================================================================================
  //=== Init OTA, MDNS, Spiffs, Web ===================================================================================
  //===================================================================================================================
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();
  MDNS.addService("http","tcp",80);
  SPIFFS.begin();
  WebSetup();
  ReadConfigFromSpiffs();
  ReadStateFromSpiffs();
  //===================================================================================================================
  //=== Init CS5460 ===================================================================================================
  //===================================================================================================================
  CS5460_init();
  //===================================================================================================================
  //=== Init  ADS1115 =================================================================================================
  //===================================================================================================================
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV

  pinMode(PIN_CS5460_INT, INPUT_PULLUP); 
  pinMode(PIN_ZEROCR_INT, INPUT_PULLUP); 
  
  timer1_isr_init();
  timer1_disable();
  timer1_attachInterrupt(ReadADSTimer);
  ADSStart();

  //===================================================================================================================
  //=== Start Interrupts Pin:2 CS5460Ready =  Pin:0 ZeroCross =========================================================
  //===================================================================================================================
  attachInterrupt(digitalPinToInterrupt(PIN_CS5460_INT), CS5460Ready, FALLING); // Intterrupt GPIO0 (D4 on Wemos D1)  CHANGE | RISING 
  attachInterrupt(digitalPinToInterrupt(PIN_ZEROCR_INT), ZeroCross, FALLING); // Intterrupt GPIO0 (D3 on Wemos D1)  CHANGE | RISING 

  last5000 = millis();
  last1000 = millis();
  timeClient.begin();
  //ThingSpeak.begin(TS_client);  // Initialize ThingSpeak
  LastMeanZeroCross = MeanZeroCross = LastZeroCross = TZeroCross = micros64();
  #ifdef SER_DBG_ON
    SER_DBG.println("Einde Setup");
  #endif
  //===================================================================================================================
  //=== Async http req setup ==========================================================================================
  //===================================================================================================================
  NRM_http_req.setDebug(false);
  NRM_http_req.setTimeout(5);
  NRM_http_req.onReadyStateChange(NRMreqResponse);
  NRD_http_req.setDebug(false);
  NRD_http_req.setTimeout(5);
  NRD_http_req.onReadyStateChange(NRDreqResponse);
  NRE_http_req.setDebug(false);
  NRE_http_req.setTimeout(5);
  NRE_http_req.onReadyStateChange(NREreqResponse);
  PVO_http_req.setDebug(false);
  PVO_http_req.setTimeout(5);
  PVO_http_req.onReadyStateChange(PVOreqResponse);
  //===================================================================================================================
  //=== End SETUP =====================================================================================================
  //===================================================================================================================
  SendEvent2nodered("Einde Setup - " + WiFi.localIP().toString());
}

//#####################################################################################################################
//##### Process TZeroCross measurement ################################################################################
//#####################################################################################################################
void ZCProcess()  {
  LastCycle = TZeroCross - LastZeroCross;
  MeanZeroCross = LastMeanZeroCross + CycleAvg; 
  if ((LastCycle < 21000) && (LastCycle > 19000))  {    // We missed a cycle - or spike - do not process
    if ((TZeroCross-MeanZeroCross) > 5000 ) {
        MeanZeroCross = TZeroCross; 
    } else {                                            // TODO optimize 
      if (MeanZeroCross > TZeroCross) {
          MeanZeroCross-= sqrt(MeanZeroCross - TZeroCross); 
      } else {
          MeanZeroCross+= sqrt(TZeroCross - MeanZeroCross); 
      }
    }
    CycleSum-=Cycle[CyclePnt];
    Cycle[CyclePnt] = LastCycle;
    CycleSum+=LastCycle;
    ++CyclePnt&=0x3F;        // Next in circular buffer (0-63)
    CycleAvg = CycleSum>>6;  //64
  }
  LastZeroCross = TZeroCross;
  LastMeanZeroCross = MeanZeroCross;
  TZeroCross = 0;
}

// ================================================================================================
// ====== DoTik1 ====== Elke +/- 1000 milliseconden ===============================================
// ================================================================================================
void DoTik1() {
  Frequency = 1000000.00 / CycleAvg;
  #ifdef SER_DBG_ON
    SER_DBG.print("1");
  #endif
  timeClient.update();
  if (ADSCount == 0) {   
    //LDebugADSVal[ADSChannel]="";
    //LDebugADSVal[ADSChannel]+="0;";
    //LDebugADSMcr[ADSChannel]+=String((long)(ADS_ZeroCross[ADSChannel] & 0x7FFFFFFFL))+";";
    int64_t sum = 0;
    byte cnt = 32;
    while (cnt > 0) {
      cnt--;
      sum += sq(ADSValue[cnt]);
      //LDebugADSVal[ADSChannel]+= ADSValue[cnt];
      //LDebugADSMcr[ADSChannel]+= (long)(ADSClock[cnt] & 0x7FFFFFFFL);
      //LDebugADSVal[ADSChannel]+= ";";
      //LDebugADSMcr[ADSChannel]+= ";";
    }
    //DebugADSVal += sum;
    //sum = sum >> 5;  // /32
    //ADS_Current[ADSChannel] = float(sqrt(sum)-ADS_Cur_Offset[ADSChannel])/ADS_Cur_Divide[ADSChannel];
    ADS_Current[ADSChannel] = sqrt(sum)/ADS_Cur_Divide[ADSChannel];
    float Power = float(CS5460Voltage)/CS5460VtDivide * ADS_Current[ADSChannel];
    ADS_Appr_Power[ADSChannel] += Power;
    ADS_Real_Power[ADSChannel] += Power * ADS_CosPhiAvg[ADSChannel];
    ADS_PowerCountMn[ADSChannel]++;
    // Start nieuwe meting op ander kanaal
    ADSStart();
  }
  lastLoopCount = LoopCount;
  LoopCount = 0;
  numEvents = 0;
  last1000 = millis();
}
// ================================================================================================
// ====== DoTik5 ====== Elke +/- 5000 milliseconden ===============================================
// ================================================================================================
void DoTik5() {
  last5000 = millis();
}

// ================================================================================================
// ====== Do1Min ====== Each minute (if NTP ok) ===================================================
// ================================================================================================
void Do1Min() {
  lastMinute = timeClient.getMinutes();
  if (CS5460CountMn > 0) {
    HouseVoltageMn = float(CS5460VoltageMn/CS5460CountMn)/CS5460VtDivide;
    HouseCurrentMn = float(CS5460CurrentMn-CS5460CtOffset)/CS5460CtDivide/CS5460CountMn;
    HousePowerMn = float(CS5460PowerMn)/CS5460PwDivide/CS5460CountMn;
    if (CS5460CountMn < 57) SendEvent2nodered("CS5460 Metingen:"+String(CS5460CountMn));
    if ((HouseVoltageMn > 215) and (HouseVoltageMn < 255)) {
      PowerSol1Mn = float(ADS_Real_Power[0]/ADS_PowerCountMn[0]);
      PowerSol2Mn = float(ADS_Real_Power[1]/ADS_PowerCountMn[1]);
      CS5460CountDay += CS5460CountMn;
      float cWhLastMinute = float(CS5460PowerMn)/CS5460PwDivide/CS5460CountMn/0.60;
      HouseECount += round(cWhLastMinute); // was int (cWhLastMinute + 0.5) - probleem met negatieve waarden
      ADS0ECount += round(PowerSol1Mn/0.60);
      ADS1ECount += round(PowerSol2Mn/0.60);
      HouseECountDay += round(float(CS5460PowerMn)/CS5460PwDivide/CS5460CountMn/0.060); // was int (cWhLastMinute + 0.5) - probleem met negatieve waarden
      ADS0ECountDay += round(PowerSol1Mn/0.060);
      ADS1ECountDay += round(PowerSol2Mn/0.060);
      /*
      ThingSpeak.setField(1, HouseVoltageMn);
      ThingSpeak.setField(2, HouseCurrentMn);
      ThingSpeak.setField(3, HousePowerMn);
      ThingSpeak.setField(4, PowerSol1Mn);
      ThingSpeak.setField(5, PowerSol2Mn);
      ThingSpeak.setField(6, float(0.0));
      ThingSpeak.setField(7, float(0.0));
      ThingSpeak.writeFields(MnChannelNumber, MnWriteAPIKey);
      */
      //if(x == 200){
      //  SER_DBG.println("Channel update successful.");
      //}
      //else{
      //  SER_DBG.println("Problem updating channel. HTTP error code " + String(x));
      //}
      yield();
      send2nodered();
      MinuteCount++;
      if (MinuteCount == 15) {
        yield();
        WriteStateToSpiffs();
        MinuteCount = 0;
      }
    } else {
      CS5460_CheckState();
      // CS5460_spi_start();
      // CS5460Status = CS5460_readReg(CS5460_REG_STATUS);  //powerMeter.getStatus();
      // SPI.endTransaction();
      // SendEvent2nodered("CS5460 Restart Stat=" + String(CS5460Status));
      // CS5460_re_init();
    }
  } else {
    if (millis() > 40000) {
      CS5460_CheckState();
    }
  }
  //int tmp = lastMinute;
  if ((lastMinute % 5) == 0) SendData2PVOutput();
  LastCountMn = CS5460CountMn;
  CS5460CountMn = 0;
  CS5460PowerMn = 0;
  CS5460VoltageMn = 0;
  CS5460CurrentMn = 0;
  ADS_PowerCountMn[0] = 0;
  ADS_PowerCountMn[1] = 0;
  ADS_Appr_Power[0] = 0;
  ADS_Appr_Power[1] = 0;
  ADS_Real_Power[0] = 0;
  ADS_Real_Power[1] = 0;
}  

// ================================================================================================
// ====== Do1Hour ====== Each hour (if NTP ok) ====================================================
// ================================================================================================
void Do1Hour() {
  lastHour = timeClient.getHours();
}
// ================================================================================================
// ====== Do1Day ====== Each day (if NTP ok) ======================================================
// ================================================================================================
void Do1Day() {
  if (CS5460CountDay > 43200) {  //minstens 12u = 12*3600 
    /*
    ThingSpeak.setField(1, float(EneHouseHr/60.0));
    ThingSpeak.setField(2, float(EneSol1Hr/60.0));
    ThingSpeak.setField(3, float(EneSol2Hr/60.0));
    ThingSpeak.setField(4, float(0.0));
    ThingSpeak.setField(5, float(0.0));
    ThingSpeak.setField(6, float(0.0));
    ThingSpeak.setField(7, float(0.0));
    ThingSpeak.writeFields(HrChannelNumber, HrWriteAPIKey);
    */
    send2noderedDay();
    SendEvent2nodered("Dagmetingen doorgestuurd:" + String(CS5460CountDay));
  } else {
    SendEvent2nodered("Te weinig metingen dag:" + String(CS5460CountDay));
  }
  if (millis() > 100000) {
    HouseECountDay = 0;
    ADS0ECountDay = 0;
    ADS1ECountDay = 0;
    CS5460CountDay = 0;
  }
  lastDay = timeClient.getDay();
}

//#####################################################################################################################
//##### LOOP ##########################################################################################################
//#####################################################################################################################
//# LoopCount used to monitor load / with current setup: +/- 8000 per sec
void loop(){
  if(IRReadADS) ADSProcess();        // Read ADS1115 (timed)
  if(TZeroCross > 0) ZCProcess();    // Voltage ZeroCross detected -> Process
  yield();
  if(IRReadCS5460) CS5460Process();  // CS5460 Measurement ready -> Process
  yield();
  if ((millis()-LastCS5460) > 1200) CS5460_CheckState();  // CS5460 interrupt missed?  Check state
  yield();
  //===================================================================================================================
  //=== Timed stuff ===================================================================================================
  //===================================================================================================================
  if((millis()-last1000) > 1000) DoTik1();
  //===================================================================================================================
  //=== ADS measurements each sec from 0 - 750 ms / concentrating other operations between 760 - 1000 =================
  //===================================================================================================================
  else if ((millis()-last1000) > 760) {  
    if (Q_SaveStat) {
       WriteStateToSpiffs();
       MinuteCount = 0;
       Q_SaveStat = false;
    }
    if ((millis()-last5000) > 5000) DoTik5();
    yield();
    if (timeClient.getEpochTime() > 1546300800) { // Anders geen NTP
      if (FirstNTP) {
        lastHour = timeClient.getHours();
        lastDay = timeClient.getDay();
        FirstNTP = false;
      }
      if (timeClient.getMinutes() != lastMinute) Do1Min();
      yield();
      if (timeClient.getHours() != lastHour) Do1Hour();
      yield();
      if (timeClient.getDay() != lastDay) Do1Day();
    } else {
      SendEvent2nodered("Geen NTP tijd!");
    }
  }
  LoopCount++;
  yield();
  ArduinoOTA.handle();
}