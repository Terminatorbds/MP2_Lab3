#include "Arduino.h"
#include "DCMDriverL293D.h"
#include "LiquidCrystal_PCF8574.h"
#include "Fan.h"
#include "Wire.h"
#include "RTClib.h"
#include <Wire.h>
#include <FFT.h>

#define DCMOTORDRIVERA_PIN_ENABLE1 5
#define DCMOTORDRIVERA_PIN_IN1 2
#define DCMOTORDRIVERA_PIN_IN2 3
#define DCMOTORDRIVERA_PIN_ENABLE2 6
#define DCMOTORDRIVERA_PIN_IN3 4
#define DCMOTORDRIVERA_PIN_IN4 7
#define PCFAN_PIN_COIL1 9
#define SND_DETECTOR_PIN_GATE 8
#define SND_DETECTOR_PIN_AUDIO A3
#define SND_DETECTOR_PIN_ENVELOPE A1

#define LCD_ADDRESS 0x3F
#define LCD_ROWS 2
#define LCD_COLUMNS 16
#define SCROLL_DELAY 150
#define BACKLIGHT 255

DCMDriverL293D dcMotorDriverA(DCMOTORDRIVERA_PIN_ENABLE1, DCMOTORDRIVERA_PIN_IN1, DCMOTORDRIVERA_PIN_IN2, DCMOTORDRIVERA_PIN_ENABLE2, DCMOTORDRIVERA_PIN_IN3, DCMOTORDRIVERA_PIN_IN4);
LiquidCrystal_PCF8574 lcdI2C;
Fan PCFan(PCFAN_PIN_COIL1);
RTC_DS3231 rtcDS;

const unsigned long timerInterval = 1000;
const int samplingFrequency = 2000;
const int samples = 128;
int vReal[samples];
int vImag[samples];
const int soundSensorPin = SND_DETECTOR_PIN_AUDIO;

bool isClockwise = true;
int fanSpeed = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("start");

  lcdI2C.begin(LCD_COLUMNS, LCD_ROWS, LCD_ADDRESS, BACKLIGHT);

  if (!rtcDS.begin()) {
    Serial.println("Where's the RTC?");
    while (1);
  }

  if (rtcDS.lostPower()) {
    Serial.println("no power on RTC, reset.");
    rtcDS.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

ISR(TIMER1_COMPA_vect) {
  DateTime now = rtcDS.now();

  lcdI2C.setCursor(0, 0);
  lcdI2C.print(now.hour());
  lcdI2C.print(":");
  lcdI2C.print(now.minute());
  lcdI2C.print(":");
  lcdI2C.print(now.second());

  lcdI2C.setCursor(0, 1);
  lcdI2C.print("Dir: ");
  if (isClockwise) {
    lcdI2C.print("C  ");
  } else {
    lcdI2C.print("CC ");
  }
  lcdI2C.print("Speed: ");
  if (fanSpeed == 255) {
    lcdI2C.print("Full");
  } else if (fanSpeed >= 192) {
    lcdI2C.print("3/4 ");
  } else if (fanSpeed >= 128) {
    lcdI2C.print("1/2 ");
  } else {
    lcdI2C.print("0   ");
  }

  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(soundSensorPin);
    vImag[i] = 0;
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples);
  int peakFrequency = FFT.MajorPeak(vReal, samples, samplingFrequency);
  if (peakFrequency >= 257 && peakFrequency <= 267) {
    // C4 detected, increase fan speed
    fanSpeed += 64;
    if (fanSpeed > 255) {
      fanSpeed = 255;
    }
  } else if (peakFrequency >= 435 && peakFrequency <= 445) {
    // A4 detected, decrease fan speed
    fanSpeed -= 64;
    if (fanSpeed < 0) {
      fanSpeed = 0;
    }
  }
  
  // Set motor direction and speed
  if (isClockwise) {
    dcMotorDriverA.setMotorA(fanSpeed, 1);
    dcMotorDriverA.setMotorB(fanSpeed, 0);
  } else {
    dcMotorDriverA.setMotorA(fanSpeed, 0);
    dcMotorDriverA.setMotorB(fanSpeed, 1);
  }
}

void loop() {
}
