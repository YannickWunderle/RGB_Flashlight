#include <Wire.h>
#include "fan_controller.h"
#include "fan_safety.h"
#include "led_derate.h"

#define PIN_RED 5
#define PIN_BLUE 11
#define PIN_GREEN 9
#define PIN_FAN 3
#define PIN_BRIGHTNESS A5
#define PIN_EFFECT A6
#define PIN_MODE A4
#define PIN_VOLTAGE A0
#define PIN_RELAIS 12
#define PIN_TEMP A7

#define TEMP_SAMPLE_MS 20UL
#define TEMP_MEAN_MS 1000UL
#define AVERAGE_SAMPLES 10

#define LOW_VOLTAGE 570  //18.6V --> 3.1V/Cell
#define MAX_STATE 4
#define MAXWELL 255
#define CycleTime 900
#define STROBE_MIN_PERIOD_MS 40   // 25 Hz at pot 0
#define STROBE_MAX_PERIOD_MS 1000 // 1 Hz at pot 1023
#define STROBE_ON_MS 10

// Approximate 1350 K (pot 0) to 6500 K (pot 1023); tune to the LED strip
#define FLASHLIGHT_WARM_R 255
#define FLASHLIGHT_WARM_G 98
#define FLASHLIGHT_WARM_B 0
#define FLASHLIGHT_COOL_R 255
#define FLASHLIGHT_COOL_G 249
#define FLASHLIGHT_COOL_B 253

float temp = 30;
int ButtonState = 0;
unsigned long ButtonLastTime = 0;
int state = 0;
bool poweredOff = false;
int specialEffect = 0;
int tempRaw = 0;
int fanDuty = 0;
FanController fan;
bool tempValid = false;
FanMedian5 tempMedian;
LedDerate derate;
LedPowerMeter ledPower;
unsigned long previousMillisLED = 0;
unsigned long b_red = 0;
unsigned long b_green = 0;
unsigned long b_blue = 255;
unsigned long lastCycle = 0;

int AverageBrightness[AVERAGE_SAMPLES];
int Av_index = 0;

int diagLastR = 0;
int diagLastG = 0;
int diagLastB = 0;
int diagBrightness = 0;

void setup() {
  Serial.begin(1000000);
  Serial.println(F("Lichtbox_v3 boot: red=D5 green=D9 blue=D11 mode=A4 brightness=A5 effect=A6"));

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_RELAIS, OUTPUT);

  pinMode(PIN_BRIGHTNESS, INPUT);
  pinMode(PIN_EFFECT, INPUT);
  pinMode(PIN_MODE, INPUT);
  pinMode(PIN_VOLTAGE, INPUT);
  pinMode(PIN_TEMP, INPUT);

  digitalWrite(PIN_RELAIS, HIGH);
  poweredOff = false;
  digitalWrite(PIN_TEMP, HIGH);
  digitalWrite(PIN_MODE, HIGH);
  fan.reset();
  tempValid = false;
  tempMedian.reset();
  derate.reset();
  ledPower.reset();
  analogWrite(PIN_FAN, 255);

  Serial.print(F("boot pins: mode="));
  Serial.print(digitalRead(PIN_MODE));
  Serial.print(F(" brightnessRaw="));
  Serial.print(analogRead(PIN_BRIGHTNESS));
  Serial.print(F(" effectRaw="));
  Serial.print(analogRead(PIN_EFFECT));
  Serial.print(F(" voltageRaw="));
  Serial.print(analogRead(PIN_VOLTAGE));
  Serial.print(F(" tempRaw="));
  Serial.println(analogRead(PIN_TEMP));
  Serial.println(F("boot: mode pin should read 1 released, 0 pressed"));

  analogWrite(PIN_RED, 2);
  analogWrite(PIN_BLUE, 2);
  analogWrite(PIN_GREEN, 2);
  delay(1000);
  analogWrite(PIN_RED, 0);
  analogWrite(PIN_GREEN, 0);
  analogWrite(PIN_BLUE, 0);
  analogWrite(PIN_FAN, 0);
  fanDuty = 0;

  Serial.println(F("setup complete, entering loop"));
}

void checkBatt() {
  if (analogRead(PIN_VOLTAGE) < LOW_VOLTAGE) {
    digitalWrite(PIN_RELAIS, LOW);
  }
}

// The relay feeds the box; if the Nano keeps running anyway (e.g. over USB) it stays dark until
// the next press of the mode button switches the relay back on.
void powerOff() {
  poweredOff = true;
  analogWrite(PIN_RED, 0);
  analogWrite(PIN_GREEN, 0);
  analogWrite(PIN_BLUE, 0);
  analogWrite(PIN_FAN, 0);
  fanDuty = 0;
  fan.reset();
  digitalWrite(PIN_RELAIS, LOW);
  Serial.println(F("long press: power off (relay LOW)"));
}

void powerOn() {
  poweredOff = false;
  digitalWrite(PIN_RELAIS, HIGH);
  Serial.println(F("press while off: power on (relay HIGH)"));
}

void checkTemp() {
  static bool sampledOnce = false;
  static bool sensorFault = false;
  static bool instFault = false;
  static float instC = 0;
  static unsigned long lastSampleMs = 0;
  static unsigned long windowStartMs = 0;
  static unsigned long rawSum = 0;
  static unsigned int rawCount = 0;
  unsigned long now = millis();
  if (!sampledOnce || now - lastSampleMs >= TEMP_SAMPLE_MS) {
    if (!sampledOnce) {
      windowStartMs = now;
    }
    sampledOnce = true;
    lastSampleMs = now;
    // The first conversion after switching channels reads wrong on this high-impedance sensor
    // line and the error depends on loop timing, so discard it.
    analogRead(PIN_TEMP);
    tempRaw = tempMedian.push(analogRead(PIN_TEMP));
    instC = fanRawToC(tempRaw);
    instFault = fanSensorFault(tempRaw);
    float c = instC;
    bool rawFault = instFault;
    sensorFault = rawFault || c >= FAN_TRIP_C;
    if (!rawFault) {
      rawSum += tempRaw;
      rawCount++;
      if (!tempValid) {
        temp = c;
        tempValid = true;
      }
    }
    if (now - windowStartMs >= TEMP_MEAN_MS) {
      windowStartMs = now;
      if (rawCount > 0) {
        temp = fanRawToC((float)rawSum / rawCount);
      }
      rawSum = 0;
      rawCount = 0;
    }
  }
  int duty = fan.update(temp, sensorFault, now);
  if (duty != fanDuty) {
    fanDuty = duty;
    analogWrite(PIN_FAN, fanDuty);
  }
  ledPower.tick(micros());
  derate.update(temp, instC, instFault || !tempValid, ledPower.average(), now);
}

void showLEDs() {
  diagBrightness = analogRead(PIN_BRIGHTNESS);
  unsigned long scale = 1023 - (unsigned long)diagBrightness;
  int r = scale * b_red / 1023;
  int g = scale * b_green / 1023;
  int b = scale * b_blue / 1023;
  ledPower.set((r + g + b) / 765.0f, micros());
  float s = derate.scale(ledPower.average());
  if (s < 1.0f) {
    r = r * s;
    g = g * s;
    b = b * s;
  }
  diagLastR = r;
  diagLastG = g;
  diagLastB = b;

  analogWrite(PIN_RED, r);
  analogWrite(PIN_GREEN, g);
  analogWrite(PIN_BLUE, b);
}

int checkButton() {  //0= no press, 1 = press, 2= long press
  int modeLevel = digitalRead(PIN_MODE);
  if ((ButtonState != 1) and (!modeLevel)) {
    ButtonState = 1;
    ButtonLastTime = millis();
    Serial.print(F("["));
    Serial.print(ButtonLastTime);
    Serial.println(F(" ms] MODE pin went LOW (press detected)"));
  }
  if ((ButtonState == 1) and (modeLevel)) {
    unsigned long held = millis() - ButtonLastTime;
    ButtonState = 0;
    Serial.print(F("["));
    Serial.print(millis());
    Serial.print(F(" ms] MODE pin went HIGH (released after "));
    Serial.print(held);
    Serial.println(F(" ms)"));
    if (held > 1400) {
      ButtonLastTime = millis();
      return 2;
    } else {
      return 1;
    }
  }

  return 0;
}


void rainbowReset() {
  b_red = 0;
  b_green = 0;
  b_blue = MAXWELL;
}

void rainbow(int pot) {
  int interval = pot / 4;
  if (pot > 0 && interval == 0) {
    interval = 1;
  }
  if (millis() - previousMillisLED > interval) {
    previousMillisLED = millis();
    if (b_green <= 0) {
      if (b_red < 255) {
        b_red++;
        b_blue--;
      } else {
        b_green = 1;
        b_blue = 0;
      }
      showLEDs();
      return;
    }

    if (b_blue <= 0) {
      if (b_green < 255) {
        b_green++;
        b_red--;
      } else {
        b_blue = 1;
        b_red = 0;
      }
      showLEDs();
      return;
    }

    if (b_red <= 0) {
      if (b_blue < 255) {
        b_blue++;
        b_green--;
      } else {
        b_red = 1;
        b_green = 0;
      }
      showLEDs();
      return;
    }
  }
}

void SteadyColour() {
  int colour = analogRead(PIN_EFFECT);
  if (colour <= 341) {
    b_red = map(colour, 0, 342, 0, MAXWELL);
    b_green = map(colour, 0, 341, MAXWELL, 0);
    b_blue = 0;
  } else if (colour > 341 and colour <= 682) {
    b_red = map(colour, 342, 682, MAXWELL, 0);
    b_blue = map(colour, 342, 682, 0, MAXWELL);
    b_green = 0;
  } else {
    b_blue = map(colour, 683, 1023, MAXWELL, 0);
    b_green = map(colour, 683, 1023, 0, MAXWELL);
    b_red = 0;
  } /*
  Serial.print("red: ");
  Serial.print(b_red);
  Serial.print(",green: ");
  Serial.print(b_green);
  Serial.print(",blue: ");
  Serial.println(b_blue); */
  showLEDs();
  //adjustBrightness();
}

#define STROBO_STEPS 100

static float stroboRed = 0;
static float stroboGreen = 0;
static float stroboBlue = 0;
static int stroboStep = 0;
static unsigned long stroboNextUs = 0;

void randomStroboReset() {
  stroboStep = 0;
  stroboNextUs = micros();
}

void RandomStrobo() {
  if ((long)(micros() - stroboNextUs) < 0) {
    return;
  }
  if (stroboStep == 0) {
    stroboRed = 0;
    stroboGreen = 0;
    stroboBlue = 0;
    switch (random(0, 6)) {
      case (0):
        stroboRed = 255;
        break;
      case (1):
        stroboRed = 255;
        stroboBlue = 255;
        break;
      case (2):
        stroboBlue = 255;
        break;
      case (3):
        stroboBlue = 255;
        stroboGreen = 255;
        break;
      case (4):
        stroboGreen = 255;
        break;
      case (5):
        stroboGreen = 255;
        stroboRed = 255;
        break;
    }
  }
  stroboRed *= 0.95;
  stroboGreen *= 0.95;
  stroboBlue *= 0.95;
  b_red = stroboRed;
  b_green = stroboGreen;
  b_blue = stroboBlue;
  showLEDs();
  stroboStep = (stroboStep + 1) % STROBO_STEPS;
  stroboNextUs = micros() + (unsigned long)analogRead(PIN_EFFECT) * 10;
}

void WhiteStrobe() {
  unsigned long period = map(analogRead(PIN_EFFECT), 0, 1023, STROBE_MIN_PERIOD_MS, STROBE_MAX_PERIOD_MS);
  static unsigned long flashStart = 0;
  unsigned long now = millis();
  if (now - flashStart >= period) {
    flashStart = now;
  }
  unsigned long level = (now - flashStart) < STROBE_ON_MS ? MAXWELL : 0;
  b_red = level;
  b_green = level;
  b_blue = level;
  showLEDs();
}

void Flashlight() {
  int pot = analogRead(PIN_EFFECT);
  b_red = map(pot, 0, 1023, FLASHLIGHT_WARM_R, FLASHLIGHT_COOL_R);
  b_green = map(pot, 0, 1023, FLASHLIGHT_WARM_G, FLASHLIGHT_COOL_G);
  b_blue = map(pot, 0, 1023, FLASHLIGHT_WARM_B, FLASHLIGHT_COOL_B);
  showLEDs();
}

void IndicateButton(bool longpress) {
  int r = longpress ? 100 : 0;
  int g = longpress ? 0 : 100;
  int b = 0;
  Serial.print(F("Indicate button: "));
  Serial.print(longpress ? F("long press, red=") : F("short press, green="));
  Serial.print(longpress ? r : g);
  Serial.println(F(" (100 ms)"));
  analogWrite(PIN_RED, r);
  analogWrite(PIN_GREEN, g);
  analogWrite(PIN_BLUE, b);
  delay(100);
  analogWrite(PIN_RED, 0);
  analogWrite(PIN_GREEN, 0);
  analogWrite(PIN_BLUE, 0);
}

void loop() {
  if (poweredOff) {
    if (checkButton() != 0) {
      powerOn();
    }
    return;
  }
  checkTemp();
  checkBatt();
  int button = checkButton();
  if (button == 1) {
    IndicateButton(0);
    int oldState = state;
    if (state < MAX_STATE) {
      state++;
    } else {
      state = 0;
    }
    if (state == 2) {
      rainbowReset();
    }
    if (state == 1) {
      randomStroboReset();
    }
    Serial.print(F("state "));
    Serial.print(oldState);
    Serial.print(F(" -> "));
    Serial.println(state);
  }
  if (button == 2) {
    IndicateButton(1);
    powerOff();
    return;
  }
  if ((millis() - lastCycle) > CycleTime) {
    lastCycle = millis();
    Serial.print(F("Temp: "));
    Serial.println(temp);
    Serial.print(F("tempRaw: "));
    Serial.print(tempRaw);
    Serial.print(F(" fanDuty: "));
    Serial.print(fanDuty);
    Serial.print(F(" ledDemand: "));
    Serial.print(ledPower.average(), 3);
    Serial.print(F(" ledCap: "));
    Serial.print(derate.cap, 3);
    Serial.println(derate.tripped ? F(" TRIPPED") : F(""));
    Serial.print(F("ModeButton: "));
    Serial.println(digitalRead(PIN_MODE));
    Serial.print(F("State: "));
    Serial.println(state);
    Serial.print(F("brightnessRaw: "));
    Serial.print(diagBrightness);
    Serial.print(F(" effectRaw: "));
    Serial.print(analogRead(PIN_EFFECT));
    Serial.print(F(" pwm: "));
    Serial.print(diagLastR);
    Serial.print('/');
    Serial.print(diagLastG);
    Serial.print('/');
    Serial.println(diagLastB);
    Serial.print(F("buttonState: "));
    Serial.print(ButtonState);
    Serial.print(F(" b_red/green/blue: "));
    Serial.print(b_red);
    Serial.print('/');
    Serial.print(b_green);
    Serial.print('/');
    Serial.println(b_blue);
  }

  switch (state) {
    case 0:
      SteadyColour();
      break;
    case 1:
      RandomStrobo();
      break;
    case 2:
      rainbow(analogRead(PIN_EFFECT));
      break;
    case 3:
      WhiteStrobe();
      break;
    case 4:
      Flashlight();
      break;
    default:
      SteadyColour();
      break;
  }
}
