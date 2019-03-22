#include "panstamp.h"
#include "product.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define RFCHANNEL        0x01       // Let's use channel 1
#define SYNCWORD1        0xB0    // Synchronization word, high byte
#define SYNCWORD0        0x48    // Synchronization word, low byte
#define SOURCE_ADDR      2       // Sender address
#define DESTINATION_ADDR 1       // Receiver address

#define motionPin 3
#define ds18b20Pin 4
#define dhtPin 7//for future realization
#define out1 5
#define out2 6
#define out3 8
#define out4 9

#define dataFlag 10
#define stateFlag 20
#define commandFlag 30

CCPACKET txPacket;  // packet object
CCPACKET rxPacket;

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ds18b20Pin);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//boolean packetAvailable = false;

long delaytime = 1;

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


void txData() {
  txPacket.data[1] = dataFlag;//mark packet as data
  txPacket.data[2] = sizeof(p); //Third data byte is length of data values
  memcpy(txPacket.data + 3 * sizeof(byte), &p, txPacket.data[2]);
  txPacket.length = 3 + txPacket.data[2]; //
  panstamp.cc1101.sendData(txPacket);     // Transmit packet
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

void txState() {
  txPacket.data[1] = stateFlag;//mark packet as state
  txPacket.data[2] = sizeof(state); //Second data byte is length of data values
  memcpy(txPacket.data + 3 * sizeof(byte), &state, txPacket.data[2]);
  txPacket.length = 3 + txPacket.data[2]; //
  panstamp.cc1101.sendData(txPacket);     // Transmit packet
}

void apllyState() {
  digitalWrite(out1, state.dout1);
  digitalWrite(out2, state.dout2);
  digitalWrite(out3, state.dout3);
  digitalWrite(out4, state.dout4);
  delaytime = state.sleepTime * 1000;
  state.change = true;
}


void measure() {

  sensors.requestTemperatures();
  p.DS18B20value1 = sensors.getTempCByIndex(0);
  p.DS18B20value2 = sensors.getTempCByIndex(1);
  p.DS18B20value3 = sensors.getTempCByIndex(2);
  p.DS18B20value4 = sensors.getTempCByIndex(3);
  p.DS18B20value5 = sensors.getTempCByIndex(4);

  p.vcc = readVcc();
  analogReference(DEFAULT);
  p.analog0 = analogRead(A0);
  p.analog1 = analogRead(A1);
  p.analog2 = analogRead(A2);
  
  if (!digitalRead(motionPin)) {
    p.motion = true;
  } else {
    p.motion = false;
  }
}

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void) {
  // Disable wireless reception interrupt
  detachInterrupt(0);
  state.rssi = ReadRSSI();
  state.lqi = ReadLQI();

  if (panstamp.cc1101.receiveData(&rxPacket) > 0) {
    if (!rxPacket.crc_ok) {
      state.change = false;
    } else {
      if (rxPacket.data[1] == commandFlag) {
        memcpy(&command, &rxPacket.data[3], rxPacket.data[2]);
        state.sleepTime = command.sleepTime; //ms
        state.dout1 = command.dout1;
        state.dout2 = command.dout2;
        state.dout3 = command.dout3;
        state.dout4 = command.dout4;
        apllyState();
      } else {
        state.change = false;
      }
    }
    txState();
  }
  // Enable wireless reception interrupt
  attachInterrupt(0, cc1101signalsInterrupt, FALLING);
}

void setup()
{
  panstamp.init();
  panstamp.cc1101.setCarrierFreq(CFREQ_868);
  panstamp.cc1101.setChannel(RFCHANNEL, true);
  panstamp.cc1101.setSyncWord(SYNCWORD1, SYNCWORD0, true);
  panstamp.cc1101.setDevAddress(SOURCE_ADDR, true);
  panstamp.cc1101.setTxPowerAmp(PA_LongDistance);
  //panstamp.enableRepeater(3);
  //panstamp.cc1101.disableAddressCheck();

  txPacket.data[0] = DESTINATION_ADDR;   // First data byte has to be the destination address
  pinMode(motionPin, INPUT);
  //attachInterrupt(1, motionDetected, RISING);
  sensors.begin();
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(out4, OUTPUT);
  state.sensorNode = SOURCE_ADDR;
  p.sensorNode = SOURCE_ADDR;
  state.dout1 = 0;
  state.dout2 = 0;
  state.dout3 = 0;
  state.dout4 = 0;
  state.sleepTime = 5;//seconds
  apllyState();
  attachInterrupt(0, cc1101signalsInterrupt, FALLING);
}




void loop()
{

  measure();
  txData();
  delay(delaytime);
  // For low-power applications replace "delay" by "panstamp.sleepWd(WDTO_8S)" for example;
  //panstamp.sleepWd(state.sleepTime);

  //if (!motion && digitalRead(motionPin)) {
  //attachInterrupt(1, motionDetected, RISING);
  //} else if (!digitalRead(motionPin)) {
  //motion = false;
  //}
}

