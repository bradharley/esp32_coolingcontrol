/*
  GPIO mapping

  33: 1 wire

  16- PWM to transistor for REFER control (Top)
  17- PWM to transistor for FREEEZER control (Bottom)
  XX- Refer Fan Speed
  YY- Freezer Fan Speed
  ZZ- TBD
  aa- TBD
  devc fan pins, bottom to top.  12-27,26 25  --transistor, inverted fan logic.
  hiletgo fan pins bottom to top 12, 27, 25, 32 ---no transistor, so if using invert fan logic.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <EEPROM.h>
#define EEPROM_SIZE 8 //store 4 numbers, 0-255 in 4 bytes.  Perfect for F Temperatures as INTs, 0-1 is refer, 2-3 is freezer setpoint

const char *HOSTNAME = "chilly"; // change according your setup : it is used in OTA and as MQTT identifier
const char *WiFi_SSID = "SVPerspective";
const char *WiFi_PW = "8187314277";
const char *VERSION = "{\"Version\":\"0.60\"}"; //new branch plus isotherm  
const char *STATUS_MSG = "{\"Message\":\"Refrigeration Controller\"}";

uint8_t conn_stat = 0; // Connection status for WiFi and MQTT:
/*
                                              // status |    WiFi   
                                              // -------+-------------
                                              //      0 |     up   
                                              //      1 |    down                                           
*/

//not currently used...   char msg[50];

// Add your MQTT Broker IP address, example:
//const char* MQTT_BROKER = "192.168.1.144";
const char *MQTT_BROKER = "192.168.44.3"; //change to reflect right system.  44.3 on boat
WiFiClient espClient;
PubSubClient client(espClient);
//websocket definition
const int TCP_PORT = 8888;   //Port number for tcp socket
WiFiServer server(TCP_PORT); //define the web server
WiFiClient tcpClient;        //client for tcp socket

bool tcpFirstTime; //track client initial connection for welcome message

// LED Pin
const int LED_PIN = 2; //GPIO 2 on these Hiletgo boards, doesn't exist on espressif dev boards

// the number of the pwm pin
const int REFER_PIN = 16;   // GPIO number for ReferControl
const int FREEZER_PIN = 17; // GPIO number for FreezerControl
const int REFER_FAN_PIN = 27;
const int FREEZER_FAN_PIN = 12; //disconnected
const int COMPRESSOR_BOX_FAN_PIN = 25;

// setting PWM properties
const int PWM_FREQ_COMPRESSORS = 5000;
const int PWM_FREQ_FAN = 25000;
const int REFER_PWM_CHANNEL = 0;
const int FREEZER_PWM_CHANNEL = 1;
const int REFER_FAN_PWM_CHANNEL = 2;
const int FREEZER_FAN_PWM_CHANNEL = 3;
const int COMPRESSOR_BOX_FAN_PWM_CHANNEL = 4;

const int PWM_RESOLUTION_0 = 8;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 33;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

//Timers
unsigned long lastStatus = 0; // counter in example code for conn_stat == 5
unsigned long lastTask = 0;   // counter in example code for conn_stat <> 5
unsigned long lastWiFiErr = 0;
unsigned long lastMsg = 0;
unsigned long referRunTime = 0;
unsigned long referCooldownTime = 0;
const unsigned long sensorPeriod = 10;          //timing between sensor pulls.  10s default
const unsigned long coolingPeriod = 60;         //in seconds, timing between refrigeration adjustments.  60s default
const unsigned long ceilingPeriod = 5;          //in minutes, timing between refrigeration adjustments.  5m default
const unsigned long compressorMaxRunTime = 20;  //in minutes, limit for compressor on time.  10mdefault
const unsigned long compressorCooldownTime = 2; //in minutes, cooldown after limit for compressor on time.  1mdefault

unsigned long lastSensorReading = sensorPeriod * 1000UL; //set inital value to not wait on first execute.  USE UL for unsigned long multipliction
unsigned long lastCoolingAdjustment = 0;                 //coolingPeriod * 1000UL;
unsigned long lastCeilingAdjustment = 0;                 //for adjustceiling function--duty cycle calculation.

//Start Cooling Variables
int referMode = 0;     // 0 auto, 1 auto max, 2 manual , dutycycle and compressor speed, 7 auto defrost, 8 manual defrost, 9 off
float boxTemperatureF; //inside esp32 box
float compressorBoxTemperatureF;
int compressorBoxfanspeed = 104; //default 248.  104 is quiet.  Play with noise and temp.  255 is OFF, 0 min spin

float referTemperatureEvapF;        //probe on evaporator
int referEvapTargetF = 17;          //evaporator target temperature to turn on compressor, auto adjusts.
float referTemperatureAirF;         //probe above mechanical thermostat
float referTemperatureCoreF;         //probe center of fridge.
int referSetpointF;                 // This loads from EEPROM. //target to hold
float referSetpointoffsetF = 10;    //temperature at which full cooling kicks in
const int evapMax = 30;                   //stops defrosting
const int referEvapMin = 8;               //(was 9)Not bottom of curve, but close.  Works very hard to get below 12. need to increase speed to get lower..
int referEvapSwing = 5;             //was 4, 6 too much, see how 5 works.
const int referCfloor = 120;         // 92 is approximately full speed....3500rpm  145 is 3000
int referCminspeed = 255;           //min speed which is fixed mqtt? 255/2000 200/2500 145/3000 92/3500rpm
int referCceiling = referCminspeed; //current ceiling(min speed) which is adjusted based on duty cycle
int referCvalue = 0;                //Pin value, 0 off 92 full speed ~2ma 255 low speed ~5ma
int referCspeed = 0;                //approximate RPM  //should probably eliminate and just calculate.
int referFanspeed = 0;
int referFanbasespeed;       //set base speed vi mqtt and stored in eeprom
int referDefrostTarget = 37; //38 doesn't melt enough.  Change if move probe.
                             // lower to 34 for daily, 40 is good for one big.
float freezerTemperatureF;
int freezerSetpointF;             // = 0; //This loads from EEPROM //target to hold:  0?   ESP32 ONLY
float freezerSetpointoffsetF = 5; //temperature at which full cooling kicks in
const int freezerCfloor = 92;     // 92 is approximately full speed....3500rpm  fixed
int freezerCceiling = 255;        //current ceiling(min speed) which is adjusted based on duty cycle
int freezerCminspeed = 200;       //min speed which is fixed (set by mqqt in future?)
int freezerCvalue = 0;            //Pin value
int freezerCspeed = 0;            //approximate RPM
int freezerFanspeed = 0;

float cutoffOffsetF = 0; //offset where compressors shutoff below setPointF

//arrays below use lots of global memory....
bool referDutycyclearray[1 * 3600 / coolingPeriod]; // one value per minute for X: 2 hours (7200) or 1 hr (3600)
int referDutycycleindex = 0;
int referDutycycle;                                   // Should match the initialization in setup()
bool freezerDutycyclearray[1 * 3600 / coolingPeriod]; // one value per minute for n hrs as above
int freezerDutycycleindex = 0;                        //should be same as refer.  No reason for duplicate
int freezerDutycycle;                                 // Should match the initialization in setup()
const int maxDutycycle = 65;                          //initialized as 65--WATCH***
//End cooling variables

//pre define functions
void calcDutycycle();
//

void publishmqtt()
{
  //tcpClient.print(millis());
  //tcpClient.println(": entering publishmqtt()");

  // Convert the value to a char array
  char tempString[8];
  //Serial.print("Temperature C: ");
  //Serial.println(tempString);

  dtostrf(boxTemperatureF, 1, 2, tempString); //from float to char array
  client.publish("chilly/temperatureF", tempString);

  dtostrf(referTemperatureAirF, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureF", tempString);

  float referTemperatureK = (referTemperatureAirF - 32) * 5 / 9 + 273.15;
  dtostrf(referTemperatureK, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureK", tempString);

  dtostrf(referTemperatureCoreF, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureCoreF", tempString);

  float referTemperatureCoreK = (referTemperatureCoreF - 32) * 5 / 9 + 273.15;
  dtostrf(referTemperatureK, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureCoreK", tempString);

  dtostrf(referTemperatureEvapF, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureEvapF", tempString);

  float referTemperatureEvapK = (referTemperatureEvapF - 32) * 5 / 9 + 273.15;
  dtostrf(referTemperatureEvapK, 1, 2, tempString); //from float to char array
  client.publish("chilly/refrigerator/temperatureEvapK", tempString);

  itoa(referCvalue, tempString, 10); //from integer to char array
  client.publish("chilly/refrigerator/compressorPinValue", tempString);

  itoa(referCspeed, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/compressor_speed", tempString);

  itoa(referDutycycle, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/dutycycle", tempString);

  itoa(referSetpointF, tempString, 10); //from float to char array
  client.publish("chilly/refrigerator/setPointF", tempString);

  itoa(referFanspeed, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/fanSpeed", tempString);

  itoa(referFanbasespeed, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/fanBaseSpeed", tempString);

  itoa(compressorBoxfanspeed, tempString, 10); //from integet to char array
  client.publish("chilly/compressorBoxFanSpeed", tempString);

  dtostrf(compressorBoxTemperatureF, 1, 2, tempString); //from float to char array
  client.publish("chilly/compressorBoxTemperatureF", tempString);

  itoa(referMode, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/mode", tempString);

  itoa(referDefrostTarget, tempString, 10); //from integet to char array
  client.publish("chilly/refrigerator/defrostTarget", tempString);

  dtostrf(freezerTemperatureF, 1, 2, tempString); //from float to char array
  client.publish("chilly/freezer/temperatureF", tempString);

  float freezerTemperatureK = ((freezerTemperatureF - 32) * 5 / 9 + 273.15);
  dtostrf(freezerTemperatureK, 1, 2, tempString); //from float to char array
  client.publish("chilly/freezer/temperatureK", tempString);

  itoa(freezerCvalue, tempString, 10); //from integer to char array
  client.publish("chilly/freezer/compressorPinValue", tempString);

  itoa(freezerCspeed, tempString, 10); //from integer to char array
  client.publish("chilly/freezer/compressor_speed", tempString);

  itoa(freezerDutycycle, tempString, 10); //from integer to char array
  client.publish("chilly/freezer/dutycycle", tempString);

  itoa(freezerSetpointF, tempString, 10); //from float to char array
  client.publish("chilly/freezer/setPointF", tempString);
  //client.publish("chilly/referTemperatureEvapF", tempString,true); //last true signifies retained message
}

void getTemps()
{
  //DeviceAddress referThermometer, outsideThermometer;
  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  DeviceAddress boxThermometer = {0x28, 0xFF, 0xD1, 0x53, 0x6E, 0x18, 0x01, 0x1E};       //Family B2.  OK.
  DeviceAddress referThermometerEvap = {0x28, 0xDD, 0x64, 0x1F, 0x2F, 0x14, 0x01, 0x7E}; //mounted port side box lower mid : disabled
  DeviceAddress referThermometerCore = {0x28, 0x71, 0xFF, 0x06, 0x2F, 0x14, 0x01, 0x33};  //currently lingering middle of fridge : unused
  //DeviceAddress referThermometerAir = {0x28, 0xAA, 0x4B, 0xCD, 0x18, 0x13, 0x02, 0xDA};  //Above Mechanical Thermostat : unused
  DeviceAddress referThermometerAir = {0x28, 0xAA, 0x14, 0xBA, 0x18, 0x13, 0x02, 0x0E};  //Next to Mechanical Thermostat on IsoTherm 

  //DeviceAddress freezerThermometer = {0x28, 0xFF, 0xD1, 0x53, 0x6E, 0x18, 0x01, 0x1E};       //replace
  //DeviceAddress freezerThermometerTest = {0x28, 0xFF, 0xD1, 0x53, 0x6E, 0x18, 0x01, 0x1E};   //replace
  DeviceAddress compressorBoxThermometer = {0x28, 0x81, 0xC1, 0x26, 0x2F, 0x14, 0x01, 0x17}; //near compressors

  //tcpClient.print(millis());
  //tcpClient.println(": entering getTemps()");

  sensors.requestTemperatures();
  // delay(10); //give a little time to return.  Does not appear to help??

  //temperatureC = sensors.getTempCByIndex(0);
  int minVal = -60; // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
  int maxVal = 180;
  float temp = sensors.getTempF(compressorBoxThermometer);

  if (temp > maxVal || temp < minVal)
  { // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
    Serial.println("ERROR reading Temperature: ");
    client.publish("chilly/error", "ERROR reading compressor box Temperature");
  }
  else
  {
    compressorBoxTemperatureF = temp;
    Serial.println(compressorBoxTemperatureF);
  }

  temp = sensors.getTempF(boxThermometer);

  if (temp > maxVal || temp < minVal)
  { // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
    Serial.println("ERROR reading Temperature: ");
    client.publish("chilly/error", "ERROR reading enclosure Temperature");
  }
  else
  {
    boxTemperatureF = temp;
    Serial.println(boxTemperatureF);
  }

  temp = sensors.getTempF(referThermometerEvap);

  if (temp > maxVal || temp < minVal)
  { // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
    Serial.println("ERROR reading Temperature: ");
    client.publish("chilly/refrigerator/error", "ERROR reading Temperature");
  }
  else
  {
    referTemperatureEvapF = temp;
    Serial.println(referTemperatureEvapF);
  }

  temp = sensors.getTempF(referThermometerAir);

  if (temp > maxVal || temp < minVal)
  { // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
    Serial.println("ERROR reading Temperature: ");
    client.publish("chilly/refrigerator/error", "ERROR reading Temperature");
  }
  else
  {
    referTemperatureAirF = temp;
    Serial.println(referTemperatureAirF);
  }

  temp = sensors.getTempF(referThermometerCore);

  if (temp > maxVal || temp < minVal)
  { // range -55 to 125 C.  retry.   Getting -127C/196.60F and sometimes zero, apparent timing after "sensors.requestTemperatures()"
    Serial.println("ERROR reading Temperature: ");
    client.publish("chilly/refrigerator/error", "ERROR reading Temperature");
  }
  else
  {
    referTemperatureCoreF = temp;
    Serial.println(referTemperatureCoreF);
  }
}

uint8_t findDevices(int pin)
{
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;
  sensors.requestTemperatures();

  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do
    {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10)
          Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7)
          Serial.print(", ");
        //repeat for tcp stream
        tcpClient.print("0x");
        if (address[i] < 0x10)
          tcpClient.print("0");
        tcpClient.print(address[i], HEX);
        if (i < 7)
          tcpClient.print(", ");
      }
      Serial.print(", Temperature in F: ");
      Serial.print(sensors.getTempFByIndex(count - 1));
      Serial.println("  },");
      //repeat for tcp stream
      tcpClient.print(", Temperature in F: ");
      tcpClient.print(sensors.getTempFByIndex(count - 1));
      tcpClient.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }
  return count;
}

void coolingControl()
{ //<------------------------------------------------------COOLING CONTROL
  /*
Philosophy is to:
1. control temperature by maintaining a somewhat constant evaporator
temperature while avoiding defrost and melting in refer/freezer.
2. Turn non based on evaporator temperature, not air temperature, and adjust evaporator
based on air temp.
*/

  tcpClient.print(millis());
  tcpClient.print(": entering cooling control. ");
  tcpClient.print("Air:");
  tcpClient.print(referTemperatureAirF);
  tcpClient.print(", Setpoint: ");
  tcpClient.print(referSetpointF);
  tcpClient.print(", Evap:");
  tcpClient.print(referTemperatureEvapF);
  tcpClient.print(", EvapTarget: ");
  tcpClient.print(referEvapTargetF);
  tcpClient.print(",referEvapSwing: ");
  tcpClient.print(referEvapSwing);
  if (referRunTime != 0)
  {
    tcpClient.print(",referRunTime: ");
    tcpClient.print(millis() - referRunTime);
    tcpClient.print("ms");
  }
  if (referCooldownTime != 0)
  {
    tcpClient.print(", referCooldownTime: ");
    tcpClient.print(millis() - referCooldownTime);
    tcpClient.print("ms");
  }
  tcpClient.println();
  if (referMode == 0 || referMode == 1)
  {
    //STARTUP LOGIC
    if (referCvalue == 0 && millis() - referCooldownTime >= compressorCooldownTime * 60 * 1000 && (referTemperatureEvapF >= referEvapTargetF || referTemperatureEvapF >= evapMax)) // || referTemperatureAirF - 1 >= referSetpointF
    {
      //turn on and set back to lowest value to protect against high startup pressure. .5 on air temp avoids short starts right above setpoint.
      // Todo--assess behavior.  Maybe remove entirely air >= setpoint.   causes 1m spikes...

      referRunTime = millis(); //set counter for runtime
      referCooldownTime = 0;   //zero counter for cooldown

      Serial.print("turning on compressor \n");
      tcpClient.print("turning on compressor because ");
      if (referTemperatureEvapF >= referEvapTargetF)
      {
        tcpClient.print("evaporator is at top target ");
      }
      /* temp marked with above
      if (referTemperatureAirF - 1 >= referSetpointF) //pair number with uppper level if statement.
      {
        tcpClient.print("air temp is >= 1 degreeF above setpoint ");
      }*/
      if (referTemperatureEvapF >= evapMax)
      {
        tcpClient.print("evaporator is at max temp");
      }
      tcpClient.println();
      referCvalue = 255; //turn on slowest speed to avoid startup load
      // referFanspeed = referFanbasespeed; //-cool 1 min before turning on fan.  Turn up the fan to default run speed.  96 ran fine , try 128 with smaller fan..   52 minimum?
      if (referTemperatureAirF < referSetpointF && referEvapTargetF < evapMax) //
      {
        referEvapTargetF = referEvapTargetF + 1;
        tcpClient.print("Starting Below Setpoint, evap too cold: increasing evap target to: ");
        tcpClient.println(referEvapTargetF);
      }
      //logic to change swing if close to evapMax and reduce cooling.  Wintertime
      if (referTemperatureAirF + 1 <= referSetpointF && referEvapTargetF == evapMax) //ex air 36 setpoint 37 and target at evapMax
      {
        if (referEvapSwing != 2)
        {
          tcpClient.println("Setting evapSwing to 2.  Too cold in here.");
          referEvapSwing = 2;
        }
        else
        {
          tcpClient.println("Keeping evapSwing at 2.  Too cold in here.");
        }
        referEvapSwing = 2;
      }
      else
      {
        if (referEvapSwing != 4)
        {
          tcpClient.println("Resetting evapSwing to 4.  Normal cooling needed.");
          referEvapSwing = 4;
        }
        else
        {
          tcpClient.println("Keeping evapSwing at 4.  Normal cooling needed.");
        }
        referEvapSwing = 4;
      }

      //alt test
      //referCceiling = 255;  //alt test.   Should reduce speed while starting below setpoint
    }
    else
    {
      //RUN LOGIC
      if (referCvalue != 0)
      { //Compressor is ON for remaining block.
        if (referTemperatureAirF > referSetpointF + referSetpointoffsetF && referMode != 1)
        { //too hot, full cooling //comment out totally for testing, will only run 2krpm while hot
          referCvalue = referCfloor;
          referCceiling = referCfloor;
          Serial.println("to hot!!!  full speed");
          client.publish("chilly/refrigerator/lastError", "refer too hot", true);
          tcpClient.println("to hot!!!  full speed");
          referFanspeed = referFanbasespeed; //hold at base speed, default 96 changeable
          //referFanspeed = 192;               //Turn up the fan to default max speed.//Freezing???
          //referEvapTargetF = referEvapTargetF - 10; // Assess if necessary
          referMode = 1; //set to max cooling
        }
        else
        {                                    //not too hot, normal cooling
          referCvalue = referCceiling;       //set back to ceiling based on duty cycle.  Pair with initial slow startup above
          referFanspeed = referFanbasespeed; //hold at base speed, default 96 changeable
          if (referMode == 1)
          {
            //referEvapTargetF = referSetpointF - 18; //Assess if necessary.
            referMode = 0;
          }
        }

        //SHUTDOWN LOGIC
        if ((referTemperatureEvapF <= referEvapTargetF - referEvapSwing || referTemperatureEvapF < referEvapMin) || (referTemperatureAirF <= referSetpointF - .25 && referTemperatureEvapF < evapMax - 2))
        // The last or shuts off early if it's fairly below set temperature and the evap temperature is mid swing.   Should stop short cycling the compressor...  1-7 late.  below early try and works.
        //if ((referTemperatureEvapF <= referEvapTargetF - referEvapSwing || referTemperatureEvapF < referEvapMin))//mod 1-7 removing last or.   See what this does...  added -.25 and changed -1 to -2 on max...

        // Air add -.25 on setpoint.  Seems to cycle on and off and change evaptarget several degrees rapidly.
        { //<12.5 works fine.  lowering to 12.0, fine, 11.5, fine, 10--ok.   Drive down to 9 here and 5lines below
          Serial.print("at or below cutoff, turning off compressor \n");
          tcpClient.print("at or below cutoff, turning off compressor because ");
          if (referTemperatureEvapF <= referEvapTargetF - referEvapSwing)
          {
            tcpClient.print("evaporator is at bottom target. ");
          }
          if (referTemperatureEvapF <= referEvapMin)
          {
            tcpClient.println("evaporator is at lowest allowed temp. ");
          }
          if (referTemperatureAirF <= 34)
          {
            tcpClient.println("air temp is way too cold.  Problem! ");
          }
          tcpClient.println();
          referCvalue = 0;
          referFanspeed = 0;                                                                                  //52; //slow down the fan to off
          referRunTime = 0;                                                                                   //zero counter for runtime
          if (referEvapTargetF - referEvapSwing - 2 >= referEvapMin && referTemperatureAirF > referSetpointF) //pair -2 with below to adjust appropriately
          {
            referEvapTargetF = referEvapTargetF - 2; //if turn with evap at defrost point, lower the temperature band.
            //alt test
            //referCceiling = 92;
            tcpClient.print("Stopping Above Setpoint, evap too warm: decreasing evap target to: ");
            tcpClient.println(referEvapTargetF);
          }
        }
        else //limit based on max compressor on time
        {
          if (millis() - referRunTime >= compressorMaxRunTime * 60 * 1000)
          {
            referCooldownTime = millis();
            tcpClient.print("Stopping because we exceeded max run time for the refer compressor");
            referCvalue = 0;
            referFanspeed = 0;
            referRunTime = 0; //zero counter for runtime
          }
        }
      }
    }
  }
  if (referMode == 7) //auto defrost
  {
    referCvalue = 0;
    referFanspeed = 255;
    //referFanspeed = 128;
    //referFanspeed = referFanbasespeed; //test to see how frosting works.
    if (referTemperatureEvapF > referDefrostTarget) //maybe make the 37.5 a variable
    {
      referMode = 0;
      tcpClient.println("Defrosting Complete");
    }
    else
    {
      tcpClient.println("Defrosting");
    }
  }

  if (referMode == 8) //timed defrost
  {
    referCvalue = 0;
    referFanspeed = 255;
    //referFanspeed = 128;
    //referFanspeed = referFanbasespeed; //test to see how frosting works.

    tcpClient.println("Defrosting");
  }

  if (referMode == 9)
  {
    referCvalue = 0;
    referFanspeed = 0;
    tcpClient.println("Off");
  }

  //add in lowest temp sensor logic and mimic refer.   Old code below

  //Freezer Section
  /*
  if (freezerTemperatureF > freezerSetpointF + .5 && freezerCvalue == 0)
  {
    freezerCvalue = freezerCceiling; //set back to lowest based on duty cycle speed
    // Serial.print ("turning on compressor \n");
    freezerFanspeed = 96; //Turn up the fan to default run speed.
  }
  else
  {
    if (freezerCvalue != 0)
    { //Compressor is ON for remaining function.
      if (freezerTemperatureF > freezerSetpointF + freezerSetpointoffsetF)
      { //too hot, full cooling
        freezerCvalue = freezerCfloor;
        freezerCceiling = freezerCfloor;
        Serial.print("freezer to hot!!!  full speed \n");
        client.publish("chilly/freezer/lastError", "freezer too hot", true);
        // freezerFanspeed = 255; //Turn up the fan to default run speed. //disable
      }

      if (freezerTemperatureF <= freezerSetpointF - cutoffOffsetF && freezerCvalue != 0)
      {
        // Serial.print ("at or below cutoff, turning off compressor \n");
        freezerCvalue = 0;
        freezerFanspeed = 0; //32; //slow down the fan to min speed.
      }
    }
  }
  */
  //Refer Duty cycle stuff
  if (referCvalue == 0)
  {
    referDutycyclearray[referDutycycleindex] = 0;
    referCspeed = 0;
  }
  else
  {
    referDutycyclearray[referDutycycleindex] = 1;
    referCspeed = 2000 + (255 - referCvalue) * 9.202454;
  }
  // Serial.print("referDutycycleindex=");
  // Serial.println(referDutycycleindex);
  if (referDutycycleindex >= sizeof(referDutycyclearray) / sizeof(referDutycyclearray[0]) - 1)
  {
    referDutycycleindex = 0;
  }
  else
  {
    referDutycycleindex++;
  }

  //Freezer Duty cycle stuff
  /*
  if (freezerCvalue == 0)
  {
    freezerDutycyclearray[freezerDutycycleindex] = 0;
    freezerCspeed = 0;
  }
  else
  {
    freezerDutycyclearray[freezerDutycycleindex] = 1;
    freezerCspeed = 2000 + (255 - freezerCvalue) * 9.202454;
  }
  // Serial.print("freezerDutycycleindex=");
  // Serial.println(freezerDutycycleindex);
  if (freezerDutycycleindex >= sizeof(freezerDutycyclearray) / sizeof(freezerDutycyclearray[0]) - 1)
  {
    freezerDutycycleindex = 0;
  }
  else
  {
    freezerDutycycleindex++;
  }
  */

  /* //debug
  tcpClient.print(millis());
  tcpClient.print(": referCvalue: ");
  tcpClient.println(referCvalue);
  */
}

void adjustCeiling()
{ //do this every N minutes, default 5.
  //tcpClient.print(millis());
  //tcpClient.println(": entering adjustCeiling()");
  //refer section

  if (referDutycycle <= maxDutycycle)
  {
    if (referCceiling <= referCminspeed - 55)
    {
      referCceiling += 55; //lower min RPM due to duty cycle  Case under duty cycle celieng <= 200, add 55 to ceiling on and off
      // Serial.print("Slowing compressor, under duty cycle limit");
      if (referCvalue <= referCceiling - 55 && referCvalue != 0)
      {
        referCvalue += 55; //only when on, as don't want to increas pin value when off....
      }
    }
    else
    { //case under duty cycle, ceiling >200
      referCceiling = referCminspeed;
    }
  }
  else
  { //case for block, over duty cycle
    if (referCceiling > referCfloor && referCvalue != 0)
    {
      if (referCvalue >= referCfloor + 55)
      {
        referCceiling -= 55; ///case over duty cycle, refer on, lower ceiling(increase cooling).
        referCvalue -= 55;   //case over duty cycle, refer on, lower pinvalue.
        // Serial.print("Increasing compressor speed, over duty cycle limit");
      }
      else
      { //refer over duty cycle, on  and Ceiling !>= 92+55
        referCceiling = referCfloor;
        referCvalue = referCfloor;
        // Serial.println("setting all values to floor (92), max speed");
      }
    }
  }

  //freezer section
  if (freezerDutycycle <= maxDutycycle)
  {
    if (freezerCceiling <= freezerCminspeed - 55)
    {
      freezerCceiling += 55; //lower min RPM due to duty cycle  Case under duty cycle celieng <= 200, add 55 to ceiling on and off
      // Serial.print("Slowing compressor, under duty cycle limit");
      if (freezerCvalue <= freezerCceiling - 55 && freezerCvalue != 0)
      {
        freezerCvalue += 55; //only when on, as don't want to increas pin value when off....
      }
    }
    else
    { //case under duty cycle, ceiling >200
      freezerCceiling = freezerCminspeed;
    }
  }
  else
  { //case for block, over duty cycle
    if (freezerCceiling > freezerCfloor && freezerCvalue != 0)
    {
      if (freezerCvalue >= freezerCfloor + 55)
      {
        freezerCceiling -= 55; ///case over duty cycle, freezer on, lower ceiling(increase cooling).
        freezerCvalue -= 55;   //case over duty cycle, freezer on, lower pinvalue.
        // Serial.print("Increasing compressor speed, over duty cycle limit");
      }
      else
      { //freezer over duty cycle, on  and Ceiling !>= 92+55
        freezerCceiling = freezerCfloor;
        freezerCvalue = freezerCfloor;
        // Serial.println("setting all values to floor (92), max speed");
      }
    }
  }
  // Serial.print("Ceiling is: ");
  // Serial.println(referCceiling);
  // Serial.print("Floor is: ");
  // Serial.println(referCfloor);
}

void calcDutycycle()
{ //int for value returning function   Try passing the array to do it for freezer and refer.
  //calculate duty cycle for refer
  //tcpClient.print(millis());
  //tcpClient.println(": entering calcDutycycle()");
  int index;
  int arraySum;

  //Refer
  arraySum = 0;
  for (index = 0; index < sizeof(referDutycyclearray) / sizeof(referDutycyclearray[0]); index++)
  {
    arraySum += referDutycyclearray[index];
  }
  referDutycycle = 100 * arraySum / (sizeof(referDutycyclearray) / sizeof(referDutycyclearray[0])); //turn to percent

  //Freezer
  arraySum = 0;
  for (index = 0; index < sizeof(freezerDutycyclearray) / sizeof(freezerDutycyclearray[0]); index++)
  {
    arraySum += freezerDutycyclearray[index];
  }
  freezerDutycycle = 100 * arraySum / (sizeof(freezerDutycyclearray) / sizeof(freezerDutycyclearray[0])); //turn to percent
}

void handleTCP()
{
  //start tcp testing
  //need server.begin in network and startup and variables tcpFirstTime etc
  //  Serial.print("loop start tcpClient.connected(): ");
  //  Serial.println(tcpClient.connected());
  if (!tcpClient.connected())
  {
    tcpClient = server.available(); //constantly keep available
    tcpFirstTime = 1;
    //Serial.println("in server.available loop");
  }

  if (tcpClient)
  {
    if (tcpFirstTime)
    {
      tcpClient.println("Welcome Brad");
      Serial.println("Client Connected");
      tcpFirstTime = false;
    }
    /*
      if (tcpClient.connected())
      {
      Serial.println("Client Connected");
      Serial.print("tcpClient: ");
      Serial.println(tcpClient);
      Serial.print("tcpClient.available(): ");
      Serial.println(tcpClient.available());
      Serial.print("tcpClient.connected(): ");
      Serial.println(tcpClient.connected());
      }
    */
    if (tcpClient.connected())
    {
      if (tcpClient.available() > 0)
      {
        // read data from the connected client
        Serial.write(tcpClient.read());
      }
      //Send Data to connected client
      if (Serial.available() > 0)
      {
        tcpClient.write(Serial.read());
      }
      //tcpClient.println("Welcome!!!");
    }
    else
    {
      tcpClient.stop();
      Serial.println("Client disconnected");
    }
  }
  //end tcp testing
}

void callback(char *topic, byte *message, unsigned int length)
{
  tcpClient.print("Message arrived on topic: ");
  tcpClient.print(topic);
  tcpClient.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    tcpClient.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  tcpClient.println();

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  /*String messageTemp;  //all redundant with processing for tcpClient above

    for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
    }
  */
  Serial.print(messageTemp);
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic chilly/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "chilly/output")
  {
    Serial.print("Changing output to ");
    if (messageTemp == "on")
    {
      Serial.println("on");
      digitalWrite(LED_PIN, HIGH);
    }
    else if (messageTemp == "off")
    {
      Serial.println("off");
      digitalWrite(LED_PIN, LOW);
    }
    else if (messageTemp == "sleep")
    {
      Serial.println("sleep");
      digitalWrite(LED_PIN, LOW);
      btStop();
      WiFi.mode(WIFI_OFF);                         ///test power changes
      esp_sleep_enable_timer_wakeup(10 * 1000000); //microsecond
      esp_deep_sleep_start();                      //deep 8ma,light 16ma at 8,39v = .065w/.1342w.  40-70 avg at 8.39v = .46w normal.
      //delay(10000); ///test power changes
    }
    else if (messageTemp == "reboot")
    {
      Serial.println("rebooting in 5s.....");
      delay(5000);
      ESP.restart();
    }
    else if (messageTemp == "scan")
    {
      Serial.println("scan");
      delay(100);
      findDevices(oneWireBus);
    }

    else if (messageTemp == "eeprom")
    {
      EEPROM.write(0, 252);
      EEPROM.commit();
      char tempString[8];
      itoa(EEPROM.read(0), tempString, 10); //from integet to char array
      client.publish("chilly/eeprom", tempString);

      //itoa(referCvalue, tempString, 10);   //from integet to char array
    }
  }

  if (String(topic) == "chilly/output/setCompressorBoxFanSpeed")
  {
    compressorBoxfanspeed = messageTemp.toInt();
  }

  if (String(topic) == "chilly/output/refrigerator/setMode")
  {
    referMode = messageTemp.toInt();
  }

  if (String(topic) == "chilly/output/refrigerator/setDefrostTarget")
  {
    referDefrostTarget = messageTemp.toInt();
  }

  if (String(topic) == "chilly/output/refrigerator/setFanSpeed")
  {
    referFanspeed = messageTemp.toInt();
    if (referFanspeed > 248)
    {
      referFanspeed = 248;
    }
    ledcWrite(REFER_FAN_PWM_CHANNEL, 255 - referFanspeed);
  }

  if (String(topic) == "chilly/output/refrigerator/setFanBaseSpeed")
  {
    referFanbasespeed = messageTemp.toInt();
    EEPROM.write(4, referFanbasespeed >> 8);   //(address, value(byte))
    EEPROM.write(5, referFanbasespeed & 0xFF); //(address, value(byte))
    EEPROM.commit();
  }

  if (String(topic) == "chilly/output/refrigerator/setDutyCycle")
  {
    referDutycycle = messageTemp.toInt();
  }

  if (String(topic) == "chilly/output/refrigerator/setTemp")
  {
    referSetpointF = messageTemp.toInt();
    EEPROM.write(0, referSetpointF >> 8);   //(address, value(byte))
    EEPROM.write(1, referSetpointF & 0xFF); //(address, value(byte))
    EEPROM.commit();
  }

  if (String(topic) == "chilly/output/freezer/setDutyCycle")
  {
    freezerDutycycle = messageTemp.toInt();
  }

  if (String(topic) == "chilly/output/freezer/setTemp")
  {
    //freezerSetpointF = messageTemp.toFloat();
    freezerSetpointF = messageTemp.toInt();
    EEPROM.write(2, freezerSetpointF >> 8);   //(address, value(byte))
    EEPROM.write(3, freezerSetpointF & 0xFF); //(address, value(byte))
    EEPROM.commit();
  }
}

void setup()
{
  setCpuFrequencyMhz(80); //Default is 240mhz, optionally 160 or 80.  DS18b20 periodic error at 80?

  Serial.begin(115200);
  Serial.println("Booting");

  EEPROM.begin(EEPROM_SIZE);
  referSetpointF = (EEPROM.read(0) << 8) + EEPROM.read(1);    //target to hold:  39?  ESP32 ONLY
  freezerSetpointF = (EEPROM.read(2) << 8) + EEPROM.read(3);  //target to hold:  39?  ESP32 ONLY
  referFanbasespeed = (EEPROM.read(4) << 8) + EEPROM.read(5); //target to hold:  39?  ESP32 ONLY
  // configure PWM functionalitites
  ledcSetup(REFER_PWM_CHANNEL, PWM_FREQ_COMPRESSORS, PWM_RESOLUTION_0);
  ledcSetup(FREEZER_PWM_CHANNEL, PWM_FREQ_COMPRESSORS, PWM_RESOLUTION_0);
  ledcSetup(REFER_FAN_PWM_CHANNEL, PWM_FREQ_FAN, PWM_RESOLUTION_0);
  ledcSetup(COMPRESSOR_BOX_FAN_PWM_CHANNEL, PWM_FREQ_FAN, PWM_RESOLUTION_0);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(REFER_PIN, REFER_PWM_CHANNEL);
  ledcAttachPin(FREEZER_PIN, FREEZER_PWM_CHANNEL);
  ledcAttachPin(REFER_FAN_PIN, REFER_FAN_PWM_CHANNEL);
  ledcAttachPin(COMPRESSOR_BOX_FAN_PIN, COMPRESSOR_BOX_FAN_PWM_CHANNEL);

  //Initialize as zero(255 as transistor in place) or set value if on.
  ledcWrite(REFER_PWM_CHANNEL, 255);
  ledcWrite(FREEZER_PWM_CHANNEL, 255);
  ledcWrite(REFER_FAN_PWM_CHANNEL, 255);
  ledcWrite(COMPRESSOR_BOX_FAN_PWM_CHANNEL, 255);

  //initialize duty cycle arrays
  memset(referDutycyclearray, 0, sizeof(referDutycyclearray));
  for (int i = 0; i < sizeof(referDutycyclearray) / sizeof(referDutycyclearray[0]); i += 3)
  { //   Should be 33% as initialized.
    referDutycyclearray[i] = 1;
  }
  memset(freezerDutycyclearray, 0, sizeof(freezerDutycyclearray));
  for (int i = 0; i < sizeof(freezerDutycyclearray) / sizeof(freezerDutycyclearray[0]); i += 3)
  { //   Should be 33% as initialized.
    freezerDutycyclearray[i] = 1;
  }
  calcDutycycle(); //define initial variables

  pinMode(LED_PIN, OUTPUT); //for play and test of onboard LED.   blinky
  // blink to show loaded sketch
  for (int j = 1; j <= 5; j++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }

  //initial setup of networking  --probably not needed with network function...
  WiFi.mode(WIFI_STA);
  //WiFi.setHOSTNAME(chilly);  //cant do with this library...
  WiFi.begin(WiFi_SSID, WiFi_PW);
  WiFi.waitForConnectResult(); //wait for some result before continuing
  server.begin();              //TCP server start

  ///mirrors network check in case 4.  maybe setup it all there...
  client.setServer(MQTT_BROKER, 1883);
  client.connect(HOSTNAME);
  client.subscribe("chilly/output");
  client.subscribe("chilly/output/#");
  client.publish("chilly/status", STATUS_MSG, true); //      send status to broker
  client.publish("chilly/version", VERSION, true);   //      send version to broker
  client.setCallback(callback);

  // Start the DS18B20 sensor
  sensors.begin();
  sensors.setResolution(12); //9-12.   at 11, lowest temp 10.06.   Trying 12-same.10.06.
  getTemps();                //initial pull of temperatures to ensure we have the data on first publish

  // initial set of referEvapTarget based on current temps
  if (referTemperatureAirF < referSetpointF + .75) //case restarting close to temp. ex. air 38.74 setpoint 38
  {
    referEvapTargetF = 25;                         // else default 20. This is a Guess.   Maybe more logic?
    if (referTemperatureAirF < referSetpointF - 1) //case restarting at low temp. ex. air 36.9 setpoint 38
    {
      referEvapTargetF = 30; // else default 20. This is a Guess.   Maybe more logic?
    }
  }
  // ARDUINOOTA START //
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(HOSTNAME);
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]()
               {
                 String type;
                 if (ArduinoOTA.getCommand() == U_FLASH)
                   type = "sketch";
                 else // U_SPIFFS
                   type = "filesystem";

                 // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                 Serial.println("Start updating " + type);
               })
      .onEnd([]()
             {
               Serial.println("\nEnd");
               ESP.restart(); //restart on upload
             })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
                 Serial.printf("Error[%u]: ", error);
                 if (error == OTA_AUTH_ERROR)
                   Serial.println("Auth Failed");
                 else if (error == OTA_BEGIN_ERROR)
                   Serial.println("Begin Failed");
                 else if (error == OTA_CONNECT_ERROR)
                   Serial.println("Connect Failed");
                 else if (error == OTA_RECEIVE_ERROR)
                   Serial.println("Receive Failed");
                 else if (error == OTA_END_ERROR)
                   Serial.println("End Failed");
               });

  ArduinoOTA.begin();
  // ARDUINOOTA STOP //

  Serial.println("Ready for OTA");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  // start of non-blocking connection setup section

  if (WiFi.status() == WL_CONNECTED)
  {
    conn_stat = 0; //connected
  }
  else
  {
    conn_stat = 1; //disconnected
  }

  if (conn_stat == 0) //do I also need to ensure mqtt connected?  //DEBUGGING
  {
    if (client.state() != MQTT_CONNECTED && millis() - lastWiFiErr > 30000)
    {
      //also resubscribe to mqtt.   Case network was alive, no mqtt.
      client.connect(HOSTNAME);
      client.subscribe("chilly/output");
      client.subscribe("chilly/output/#");
      lastWiFiErr = millis();
    }
    else
    {
      if (millis() - lastMsg > 5000) //5 seconds.   15s previously
      {                              // Start send status every n sec (just as an example)
        getTemps();                  //run when connnected
        lastSensorReading = millis();
        publishmqtt();
        client.loop();      //      give control to MQTT to send message to broker
        lastMsg = millis(); //      remember time of last sent status message
      }
    }
    //ArduinoOTA.handle();                                            // internal household function for OTA
    client.loop(); // internal household function for MQTT
    handleTCP();
  }
  else //wifi not connected, let's try to reconnect
  {
    if (conn_stat == 1 && millis() - lastWiFiErr > 30000) //only try to reconnect twice per minute
    {
      WiFi.disconnect();
      delay(1000); //bad form, but should be very rare
      WiFi.reconnect();
      //also resubscribe to mqtt.   Case network was alive, no mqtt.
      client.connect(HOSTNAME);
      client.subscribe("chilly/output");
      client.subscribe("chilly/output/#");
      lastWiFiErr = millis();
    }
  }

  // end of section for tasks where WiFi/MQTT are required

  // Start cooling logic
  if (millis() - lastCoolingAdjustment > coolingPeriod * 1000UL)
  { //in seconds default 60
    lastCoolingAdjustment = millis();
    coolingControl();
    calcDutycycle();
  }

  if (millis() - lastCeilingAdjustment > ceilingPeriod * 60UL * 1000UL)
  { //in Minutes   default 5.
    lastCeilingAdjustment = millis();
    adjustCeiling();
  }

  if (millis() - lastSensorReading > sensorPeriod * 1000UL)
  {
    lastSensorReading = millis(); //run when not connected to network
    getTemps();
  }
  //below reformatting needed as 255 actually turns of as transistor inverts
  if (referFanspeed > 248)
  {
    referFanspeed = 248;
  }
  if (compressorBoxfanspeed > 248)
  {
    compressorBoxfanspeed = 248;
  }
  ledcWrite(REFER_PWM_CHANNEL, referCvalue);
  ledcWrite(FREEZER_PWM_CHANNEL, freezerCvalue);
  ledcWrite(REFER_FAN_PWM_CHANNEL, 255 - referFanspeed);                  //set fan speed.  Inverted as ran through transistor
  ledcWrite(COMPRESSOR_BOX_FAN_PWM_CHANNEL, 255 - compressorBoxfanspeed); //set fan speed.  Inverted as ran through transistor

  //end cooling logic

  ArduinoOTA.handle();
}
