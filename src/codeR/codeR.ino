#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ezButton.h>
#include <EEPROM.h>
#include <IRremote.h>


#define none 2
//BLE
#define service_name "Room"
#define pop "123456789"

//timers
uint64_t w = 0;

//lightPins
#define light1Pin 17
#define light2Pin 18
#define motorPin 19
#define fanPin1 23
#define fanPin2 22
#define fanPin3 21
#define pirPin 4

//EEPROM Addr
#define light1Addr 0
#define light2Addr 1
#define motorAddr 2
#define fanAddr 3
#define fanSpeedAddr 4
#define pirAddr 5

//gpio
#define wifiLed 2
#define rst 0
#define pirInd 16
#define pirIPin 5

//deviceNames
#define light1 "Light"
#define light2 "Bed Light"
#define fan "Fan"
#define motor "Socket"
#define pir "Pir"

//DeviceStates
bool light1State = false;
bool light2State = false;
bool motorState = false;
bool fanState = false;
bool fanState1 = false;
bool fanState2 = false;
bool fanState3 = false;
uint8_t fanSpeed = 1;
bool pirState = false;
bool prevMode = false;
bool pirMode = false;
bool fanMode = false;
bool lightMode = false;

//deviceInstance
static LightBulb myLight1(light1, NULL, false);
static LightBulb myLight2(light2, NULL, false);
static Fan myFan(fan, NULL, false);
static Switch myMotor(motor, NULL, false);
static Switch myPir(pir, NULL, false);

//ezButton
#define btn1 36
#define btn2 39
#define btn3 34
#define btn4 35
#define btn5 12
ezButton button1(btn1, INPUT);
ezButton button2(btn2, INPUT);
ezButton motorBtn(btn3, INPUT);
ezButton fanBtn(btn4, INPUT);
ezButton pirBtn(btn5, INPUT);

//IrRemote
#define recvPin 27
uint32_t irData;
#define power 4244768519
#define one 4211345159
#define two 4194633479
#define three 4177921799
#define four 4144498439
#define plus 4161210119
#define minus 4094363399

void blink(uint8_t t, int speed = 100) {
  for (int i = 0; i < t; i++) {
    digitalWrite(wifiLed, LOW);
    delay(speed);
    digitalWrite(wifiLed, HIGH);
    delay(speed);
  }
}

void writeRom(uint8_t addr, bool &state) {
  EEPROM.write(addr, state);
  EEPROM.commit();
}

void controlDevice(uint8_t pin, bool &state, uint8_t addr) {
  state = !state;
  digitalWrite(pin, state);
  writeRom(addr, state);
}

#define r1 32
#define r2 33
#define r3 25
#define r4 26
struct level {
  bool cState = false;
  bool pState = true;
};

level s1;
level s2;
level s3;
level s4;

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S2
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#endif
      break;
    default:;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(device_name, light1) == 0) {
    if (strcmp(param_name, "Power") == 0) {
      light1State = !(val.val.b);
      controlDevice(light1Pin, light1State, light1Addr);
      param->updateAndReport(val);
      (light1State) ? lightMode = true : lightMode = false;
    }
  }

  if (strcmp(device_name, light2) == 0) {
    if (strcmp(param_name, "Power") == 0) {
      light2State = !(val.val.b);
      controlDevice(light2Pin, light2State, light2Addr);
      param->updateAndReport(val);
    }
  }

  if (strcmp(device_name, motor) == 0) {
    if (strcmp(param_name, "Power") == 0) {
      motorState = !(val.val.b);
      controlDevice(motorPin, motorState, motorAddr);
      param->updateAndReport(val);
    }
  }

  if (strcmp(device_name, fan) == 0) {
    if (strcmp(param_name, "Power") == 0) {
      fanState = !(val.val.b);
      controlFan();
      (fanState) ? fanMode = true : fanMode = false;
    }
    if (strcmp(param_name, "Speed") == 0) {
      fanSpeed = val.val.i;
      fanSpeedControl();
    }
  }

  if (strcmp(device_name, pir) == 0) {
    if (strcmp(param_name, "Power") == 0) {
      pirMode = val.val.b;
      Serial.print("pirmode: ");
      Serial.println(pirMode);
      writeRom(pirAddr, pirMode);
      param->updateAndReport(val);
    }
  }
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(6);
  IrReceiver.begin(recvPin, ENABLE_LED_FEEDBACK);
  pinMode(light1Pin, OUTPUT);
  pinMode(light2Pin, OUTPUT);
  pinMode(fanPin1, OUTPUT);
  pinMode(fanPin2, OUTPUT);
  pinMode(fanPin3, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(pirInd, OUTPUT);
  pinMode(pirIPin, OUTPUT);
  digitalWrite(light1Pin, light1State);
  digitalWrite(light2Pin, light2State);
  digitalWrite(motorPin, motorState);
  fanOff();
  pinMode(wifiLed, OUTPUT);
  pinMode(pirPin, INPUT);
  pinMode(rst, INPUT);
  pinMode(r1, INPUT);
  pinMode(r2, INPUT);
  pinMode(r3, INPUT);
  pinMode(r4, INPUT);

  blink(5);

  button1.setDebounceTime(100);
  button2.setDebounceTime(100);
  motorBtn.setDebounceTime(100);
  fanBtn.setDebounceTime(100);
  pirBtn.setDebounceTime(100);


  Param fanLevel("Speed", "custom.param.speed", value(1), PROP_FLAG_READ | PROP_FLAG_WRITE);
  fanLevel.addBounds(value(1), value(4), value(1));
  fanLevel.addUIType(ESP_RMAKER_UI_SLIDER);
  myFan.addParam(fanLevel);

  Node newNode;
  newNode = RMaker.initNode("Room", "Smart Home");

  myLight1.addCb(write_callback);
  myLight2.addCb(write_callback);
  myFan.addCb(write_callback);
  myMotor.addCb(write_callback);
  myPir.addCb(write_callback);

  newNode.addDevice(myLight1);
  newNode.addDevice(myLight2);
  newNode.addDevice(myFan);
  newNode.addDevice(myMotor);
  newNode.addDevice(myPir);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.start();

  WiFi.onEvent(sysProvEvent);

#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  light1State = EEPROM.read(light1Addr);
  light2State = EEPROM.read(light2Addr);
  motorState = EEPROM.read(motorAddr);
  fanState = EEPROM.read(fanAddr);
  fanSpeed = EEPROM.read(fanSpeedAddr);
  pirMode = EEPROM.read(pirAddr);
  prevMode = pirMode;

  digitalWrite(pirInd, pirMode);
  digitalWrite(light1Pin, light1State);
  digitalWrite(light2Pin, light2State);
  digitalWrite(motorPin, motorState);
  fanOff();
  fanSpeedControl();

  (light1State) ? lightMode = true : lightMode = false;
  (fanState) ? fanMode = true : fanMode = false;

  myLight1.updateAndReportParam("Power", light1State);
  myLight2.updateAndReportParam("Power", light2State);
  myFan.updateAndReportParam("Power", fanState);
  myMotor.updateAndReportParam("Power", motorState);
  myPir.updateAndReportParam("Power", pirMode);
}

void loop() {
  if (digitalRead(rst) == LOW) {
    Serial.printf("Reset Button Presses!\n");
    delay(100);
    int startTime = millis();
    while (digitalRead(rst) == LOW) delay(50);
    int endTime = millis();
    if ((endTime - startTime) > 10000) {
      blink(3);
      Serial.printf("Reset to factory.\n");
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      blink(5, 400);
      RMakerWiFiReset(2);
    }
  }
  delay(10);
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(wifiLed, LOW);
    if (millis() - w > 5000) {
      WiFi.reconnect();
      w = millis();
    }
  } else digitalWrite(wifiLed, HIGH);

  manualSwitchs();
  regulator();
  remote();
  pirSensor();

  if (pirMode == true && prevMode == false) {
    (pirState) ? fanState = false : fanState = true;
    (pirState) ? light1State = false : light1State = true;
    controlFan();
    controlDevice(light1Pin, light1State, light1Addr);
    myLight1.updateAndReportParam("Power", light1State);
    Serial.println("Testing Function");
  }
  if (pirMode != prevMode) prevMode = pirMode;
}

void manualSwitchs() {
  button1.loop();
  if (button1.isPressed()) {
    controlDevice(light1Pin, light1State, light1Addr);
    (light1State) ? lightMode = true : lightMode = false;
    myLight1.updateAndReportParam("Power", light1State);
  }

  button2.loop();
  if (button2.isPressed()) {
    controlDevice(light2Pin, light2State, light2Addr);
    myLight2.updateAndReportParam("Power", light2State);
  }

  motorBtn.loop();
  if (motorBtn.isPressed()) {
    controlDevice(motorPin, motorState, motorAddr);
    myMotor.updateAndReportParam("Power", motorState);
  }

  fanBtn.loop();
  if (fanBtn.isPressed()) {
    controlFan();
    (fanState) ? fanMode = true : fanMode = false;
  }

  pirBtn.loop();
  if (pirBtn.isPressed()) {
    pirMode = !pirMode;
    Serial.print("pirmode: ");
    Serial.println(pirMode);
    writeRom(pirAddr, pirMode);
    myPir.updateAndReportParam("Power", pirMode);
  }
}

//controls
void controlFan() {
  fanState = !fanState;
  (fanState) ? fanOn() : fanOff();
  myFan.updateAndReportParam("Power", fanState);
  writeRom(fanAddr, fanState);
}

void fanOff() {
  digitalWrite(fanPin1, false);
  digitalWrite(fanPin2, false);
  digitalWrite(fanPin3, false);
  delay(100);
}

void fanOn() {
  fanOff();
  digitalWrite(fanPin1, fanState1);
  digitalWrite(fanPin2, fanState2);
  digitalWrite(fanPin3, fanState3);
  delay(50);
}

void fanSpeedControl() {
  if (fanSpeed == 1) speed1();
  if (fanSpeed == 2) speed2();
  if (fanSpeed == 3) speed3();
  if (fanSpeed == 4) speed4();
  myFan.updateAndReportParam("Speed", fanSpeed);
  EEPROM.write(fanSpeedAddr, fanSpeed);
  EEPROM.commit();
}

void speed1() {
  fanState1 = true;
  fanState2 = false;
  fanState3 = false;
  if (fanState) fanOn();
}

void speed2() {
  fanState1 = false;
  fanState2 = true;
  fanState3 = false;
  if (fanState) fanOn();
}

void speed3() {
  fanState1 = true;
  fanState2 = true;
  fanState3 = false;
  if (fanState) fanOn();
}

void speed4() {
  fanState1 = false;
  fanState2 = false;
  fanState3 = true;
  if (fanState) fanOn();
}

void regulator() {
  s1.cState = digitalRead(r1);
  s2.cState = digitalRead(r2);
  s3.cState = digitalRead(r3);
  s4.cState = digitalRead(r4);
  delay(10);

  if (s1.cState && s1.pState == false) {
    fanSpeed = 1;
    fanSpeedControl();
    Serial.print("fan Speed 1");
  }

  if (s2.cState && s2.pState == false) {
    fanSpeed = 2;
    fanSpeedControl();
    Serial.print("fan Speed 2");
  }

  if (s3.cState && s3.pState == false) {
    fanSpeed = 3;
    fanSpeedControl();
    Serial.print("fan Speed 3");
  }

  if (s4.cState && s4.pState == false) {
    fanSpeed = 4;
    fanSpeedControl();
    Serial.print("fan Speed 4");
  }

  if (s1.cState != s1.pState) {
    s1.pState = s1.cState;
  }

  if (s2.cState != s2.pState) {
    s2.pState = s2.cState;
  }

  if (s3.cState != s3.pState) {
    s3.pState = s3.cState;
  }

  if (s4.cState != s4.pState) {
    s4.pState = s4.cState;
  }
}

//REMOTE CONTROL
void remote() {
  if (IrReceiver.decode()) {
    irData = IrReceiver.decodedIRData.decodedRawData;
    Serial.println(irData);
    delay(100);
    switch (irData) {
      // all off/on
      case power:
        pirMode = !pirMode;
        Serial.print("pirmode: ");
        Serial.println(pirMode);
        writeRom(pirAddr, pirMode);
        myPir.updateAndReportParam("Power", pirMode);
        break;
      // light 1
      case one:
        controlDevice(light1Pin, light1State, light1Addr);
        (light1State) ? lightMode = true : lightMode = false;
        myLight1.updateAndReportParam("Power", light1State);
        break;
      // light 2
      case two:
        controlDevice(light2Pin, light2State, light2Addr);
        myLight2.updateAndReportParam("Power", light2State);
        break;
      // fan
      case three:
        controlFan();
        (fanState) ? fanMode = true : fanMode = false;
        break;
      // shocket
      case four:
        controlDevice(motorPin, motorState, motorAddr);
        myMotor.updateAndReportParam("Power", motorState);
        break;
      case plus:
        if (fanSpeed < 4) {
          fanSpeed++;
          fanSpeedControl();
          Serial.printf("fan Speed is %d\n", fanSpeed);
        }
        break;
      case minus:
        if (fanSpeed > 1) {
          fanSpeed--;
          fanSpeedControl();
          Serial.printf("fan Speed is %d\n", fanSpeed);
        }
        break;
    }
    IrReceiver.resume();
    irData = 0;
  }
}

uint8_t pirStatus() {
  if (pirState == false) {
    if (digitalRead(pirPin) == true) {
      Serial.println("Motion Detected");
      pirState = true;
      digitalWrite(pirIPin, true);
      return true;
    }
  } else {
    if (digitalRead(pirPin) == false) {
      Serial.println("Motion End");
      pirState = false;
      digitalWrite(pirIPin, false);
      return false;
    }
  }
  return none;
}

void pirSensor() {
  uint8_t status = pirStatus();
  if (pirMode) {
    if (status != none) {
      Serial.printf("Fanmode = %d || lightMode = %d \n", fanMode, lightMode);
      if (fanMode) {
        (status) ? fanState = false : fanState = true;
        controlFan();
      }
      if (lightMode) {
        (status) ? light1State = false : light1State = true;
        controlDevice(light1Pin, light1State, light1Addr);
        myLight1.updateAndReportParam("Power", light1State);
      }
    }
    digitalWrite(pirInd, true);
  } else {
    digitalWrite(pirInd, false);
  }
}