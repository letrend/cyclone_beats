#include <Wire.h>
#include "ClosedCube_TCA9546A.h"
#include <Dps310.h>
#include <Adafruit_NeoPixel.h>
#include <Control_Surface.h> // Include the Control Surface library
#define LED_PIN        13
#define PIN            8
#define NUMPIXELS      8
#define DEADBAND 0
#define SENSITIVITY 0.01
#define PIXEL(sensor) (7-sensor)
#define KNOB_CLK 4
#define KNOB_DT 3
#define KNOB_SW 2
#define SW_0 9
#define SW_1 10

USBMIDI_Interface midi;
//USBDebugMIDI_Interface midi{115200};

// Instantiate a NoteButton object
NoteButton button {
  KNOB_SW,                       // Push button on pin 5
  {MIDI_Notes::C(4), CHANNEL_1}, // Note C4 on MIDI channel 1
};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

ClosedCube_TCA9546A tca9546a;

Dps310 sensors[8];
float pressure_mean[8] = {0};
int8_t oversampling = 0;

//uint8_t pressure_count = 0, temperature_count = 0;
//float pressure[32], temperature[32];

float measure(int sensor){
  int channel = sensor/2;
  tca9546a.selectChannel(channel);
  float pressure;
  int ret = sensors[sensor].measurePressureOnce(pressure,0);
//  int16_t ret = sensors[sensor].getContResults(&temperature[0], pressure_count, &pressure[0], temperature_count);
//  if(sensor==0){
//    Serial.println(pressure_count);
//    Serial.println(pressure[0]);
//  }
  return pressure;
}

void setup()
{
  Serial.begin(115200);
	
	tca9546a.begin(0x71);
 
  float pressure;

  int j = 0;
  for(int i=0;i<4;i++){
    tca9546a.selectChannel(i);
    sensors[j].begin(Wire, 0x76);
//    int16_t ret = sensors[j].startMeasurePressureOnce(prs_mr, prs_osr);
    sensors[j].correctTemp();
    int ret = sensors[j].measurePressureOnce(pressure,0);
    Serial.print("sensor \t");
    Serial.print(j);
    if (ret != 0){
      Serial.print("\tInit FAILED! ret = ");
      Serial.println(ret);
    }else{
      Serial.println("\tInit complete!\t");
    }

    j++;
    sensors[j].begin(Wire, 0x77);
//    ret = sensors[j].startMeasurePressureCont(prs_mr, prs_osr);
    sensors[j].correctTemp();
    ret = sensors[j].measurePressureOnce(pressure,0);
    Serial.print("sensor \t");
    Serial.print(j);
    if (ret != 0){
      Serial.print("\tInit FAILED! ret = ");
      Serial.println(ret);
    }else{
      Serial.println("\tInit complete!\t");
    }
    j++;
  }

  pixels.begin(); // This initializes the NeoPixel library.

  pinMode(LED_PIN, OUTPUT);
  
  // intialize mean for a few seconds
  unsigned long t0 = millis(), t1;
  int samples = 0;
  do{
    t1 = millis();
    for(int sensor = 0; sensor<8; sensor++){
      float pressure = measure(sensor);
      pressure_mean[sensor] += pressure;
      double counter = (t1-t0)/3000.0;
      pixels.setPixelColor(PIXEL(sensor), pixels.Color((255-(1.0-counter)*255),0,0,0));
    }
    pixels.show();
    delay(10);
    samples++;
  }while((t1-t0)<3000);
  for(int sensor = 0; sensor<8; sensor++){
    pressure_mean[sensor] /= (float)samples;
    Serial.println(pressure_mean[sensor]);
  }
  delay(500);
  Control_Surface.begin(); // Initialize Control Surface
}

void loop()
{
	for(int sensor = 0; sensor<8; sensor++){
    float pressure = measure(sensor);
    float diff = pressure - pressure_mean[sensor];
    if(diff < -DEADBAND){
      uint8_t white = min((-diff-DEADBAND)*SENSITIVITY,255);
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,0,white));
    }else if(diff>=-DEADBAND && diff<=DEADBAND) {
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,0,0));
    }else{
      uint8_t blue = min((diff-DEADBAND)*SENSITIVITY,255);
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,blue,0));
    }
//    pixels.setPixelColor(sensor, pixels.Color(0,0,0,0));
//    Serial.print(pressure_mean[sensor]); 
//    Serial.print( "\t"); 
//    Serial.println(diff); 
    
	}
  pixels.show(); // This sends the updated pixel color to the hardware.
  Control_Surface.loop(); // Update the Control Surface
}
