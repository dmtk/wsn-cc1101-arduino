
#include "panstamp.h"
#include "product.h"
#include <ArduinoJson.h>//ArduinoJson 5.13.5
#include <OneWire.h>
#include <DallasTemperature.h>


#define RFCHANNEL        0x01       // Let's use channel 0
#define SYNCWORD1        0xB0    // Synchronization word, high byte
#define SYNCWORD0        0x48    // Synchronization word, low byte
#define SOURCE_ADDR      1       // Sender address
//#define DESTINATION_ADDR 2       // Receiver address
#define ONE_WIRE_BUS 4

#define dataFlag 10
#define stateFlag 20
#define commandFlag 30

CCPACKET txPacket;  // packet object
CCPACKET rxPacket;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

typedef struct {
  byte sensorNode;
  float DS18B20value1;
  float DS18B20value2;
  float DS18B20value3;
  float DS18B20value4;
  float DS18B20value5;
  boolean motion;
  long vcc;
  int analog0;
  int analog1;
  int analog2;

} Payload;
Payload p;

typedef struct {
  int sleepTime; //ms
  int dout1;
  int dout2;
  int dout3;
  int dout4;

} Command;
Command command;

typedef struct {
  byte sensorNode;
  int sleepTime; //ms
  int dout1;
  int dout2;
  int dout3;
  int dout4;
  byte lqi;
  byte rssi;
  boolean change;

} CurrentState;
CurrentState state;

boolean packetAvailable = false;
/**
   This function is called whenever a wireless packet is received
*/

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void) {
  // set the flag that a package is available
  packetAvailable = true;
}

byte ReadLQI()
{
  byte lqi = 0;
  byte value = 0;
  lqi = (panstamp.cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
  value = 0x3F - (lqi & 0x3F);
  return value;
}

byte ReadRSSI()
{
  byte rssi = 0;
  byte value = 0;

  rssi = (panstamp.cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));

  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  return value;
}

/*Read VCC */
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void setup()
{
  Serial.begin(9600);
  // Setup LED output pin
  panstamp.init();
  panstamp.cc1101.setCarrierFreq(CFREQ_868);
  panstamp.cc1101.setChannel(RFCHANNEL, true);
  panstamp.cc1101.setSyncWord(SYNCWORD1, SYNCWORD0, true);
  panstamp.cc1101.setDevAddress(SOURCE_ADDR, true);

  attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  sensors.begin();


  // Let's disable address check for the current project so that our device
  // will receive packets even not addressed to it.
  //panstamp.cc1101.enableAddressCheck();

}







void loop()
{


  sensors.requestTemperatures();
  if (packetAvailable) {

    // Disable wireless reception interrupt
    detachInterrupt(0);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();


    byte rssi = ReadRSSI();
    byte lqi = ReadLQI();
    // clear the flag
    packetAvailable = false;


    if (panstamp.cc1101.receiveData(&rxPacket) > 0) {

      if (!rxPacket.crc_ok) {
        root["type"] = "error";
        root["cause"] = "crc not ok";
        root.printTo(Serial);

      } else {

        if (rxPacket.length > 0) {

          if (rxPacket.data[1] == dataFlag) {
            memcpy(&p, &rxPacket.data[3], rxPacket.data[2]);

            root["type"] = "data";
            root["sensorNode"] = p.sensorNode;
            root["DS18B20value1"] = p.DS18B20value1;
            root["DS18B20value2"] = p.DS18B20value2;
            root["DS18B20value3"] = p.DS18B20value3;
            root["DS18B20value4"] = p.DS18B20value4;
            root["DS18B20value5"] = p.DS18B20value5;

            root["motion"] = p.motion;
            root["vcc"] = p.vcc;
            root["analog0"] = p.analog0;
            root["analog1"] = p.analog1;
            root["analog2"] = p.analog2;
            root["rssi"] = rssi;
            root["lqi"] = lqi;
            root.printTo(Serial);
            Serial.println();

          } else if (rxPacket.data[1] == stateFlag) {

            memcpy(&state, &rxPacket.data[3], rxPacket.data[2]);
            root["type"] = "state";
            root["sensorNode"] = state.sensorNode;
            root["sleepTime"] = state.sleepTime;
            root["dout1"] = state.dout1;
            root["dout2"] = state.dout2;
            root["dout3"] = state.dout3;
            root["dout4"] = state.dout4;
            root["lqiRemote"] = state.lqi;
            root["rssiRemote"] = state.rssi;
            root["change"] = state.change;
            root["rssi"] = rssi;
            root["lqi"] = lqi;
            root.printTo(Serial);
            Serial.println();

          } else {
            root["type"] = "error";
            String str = "packet len -" + rxPacket.length;
            str += " data: ";
            for (int j = 0; j < rxPacket.length; j++) {
              str += rxPacket.data[j] + " ";

            }
            root["cause"] = str;
            root.printTo(Serial);
            Serial.println();
          }
        }
      }
    }
    // Enable wireless reception interrupt
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
  }
  delay(1);
  if (Serial.available() > 0) {
    String str = Serial.readString();
    StaticJsonBuffer<200> jsonBuffer;
    char json[str.length()];
    //{"sensorNode":3,"sleepTime":5000,"dout1":0,"dout2":0,"dout3":0,"dout4":0}
    str.toCharArray(json, sizeof(json));
    Serial.println(json);
    JsonObject& root = jsonBuffer.parseObject(json);
    Serial.println(json);
    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed");
    } else {
      // Fetch values
      // Most of the time, you can rely on the implicit casts.
      // In other case, you can do root["time"].as<long>();
      int sensorNode = root["sensorNode"];
      command.sleepTime = root["sleepTime"];
      command.dout1 = root["dout1"];
      command.dout2 = root["dout2"];
      command.dout3 = root["dout3"];
      command.dout4 = root["dout4"];
      txPacket.data[0] = sensorNode;
      txPacket.data[1] = commandFlag;//mark packet as command
      txPacket.data[2] = sizeof(command); //Third data byte is length of data values
      memcpy(txPacket.data + 3 * sizeof(byte), &command, txPacket.data[2]);
      txPacket.length = 3 + txPacket.data[2]; //
      panstamp.cc1101.sendData(txPacket);     // Transmit packet
      // Print values.
      Serial.println(sensorNode);
      Serial.println(command.sleepTime);
      Serial.println(command.dout1);
      Serial.println(command.dout2);
      Serial.println(command.dout3);
      Serial.println(command.dout4);
    }
  }
  //-------------------
  if (true) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["type"] = "data";
    root["sensorNode"] = 1;
    root["DS18B20value1"] = sensors.getTempCByIndex(0);
    root["DS18B20value2"] = sensors.getTempCByIndex(1);
    root["DS18B20value3"] = sensors.getTempCByIndex(2);
    root["vcc"] = readVcc();
    root.printTo(Serial);
    Serial.println();
  }
}

