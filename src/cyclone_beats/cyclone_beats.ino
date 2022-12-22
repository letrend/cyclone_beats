#include <Wire.h>
#include "ClosedCube_TCA9546A.h"
#include <Dps310.h>
#include <Adafruit_NeoPixel.h>
#define LED_PIN        13
#define PIN            8
#define NUMPIXELS      8
#define DEADBAND 15
#define SENSITIVITY 0.8
#define PIXEL(sensor) (7-sensor)
#define KNOB_CLK 2
#define KNOB_DT 3
#define KNOB_SW 4
#define SW_0 9
#define SW_1 10

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

ClosedCube_TCA9546A tca9546a;

Dps310 sensors[8];
int32_t pressure_mean[8];
int8_t oversampling = 0;

int32_t measure(int sensor){
  float pressure;

  int channel = sensor/2;
  tca9546a.selectChannel(channel);

//  Serial.print("Measurement sensor ");
//  Serial.println(sensor);
  
  int ret = sensors[sensor].measurePressureOnce(pressure);

  if (ret != 0)
  {
//    Serial.println();
//    Serial.println();
//    Serial.print("FAIL! ret = ");
//    Serial.println(ret);
    return 0;
  }
  else
  {
//    Serial.print(temperature);
//    Serial.println(" degrees of Celsius");
//    Serial.print(pressure);
//    Serial.println(" Pascal");
    return pressure;
  }
}

void setup()
{
	Serial.begin(9600);
	Serial.println("ClosedCube TCA9546 Arduino Test");

	// I2C address is 0x77
	tca9546a.begin(0x70);
 
  float pressure;

  int j = 0;
  for(int i=0;i<4;i++){
    tca9546a.selectChannel(i);
    sensors[j].begin(Wire, 0x76);
    sensors[j].correctTemp();
    int ret = sensors[j].measurePressureOnce(pressure);
    Serial.print("sensor \t");
    Serial.print(j);
    if (ret != 0){
      Serial.print("\tInit FAILED! ret = ");
      Serial.println(ret);
    }else{
      Serial.print("\tInit complete!\t");
      Serial.print(pressure);
      Serial.println(" Pascal");
    }

    j++;
    sensors[j].begin(Wire, 0x77);
    sensors[j].correctTemp();
    ret = sensors[j].measurePressureOnce(pressure);
    Serial.print("sensor \t");
    Serial.print(j);
    if (ret != 0){
      Serial.print("\tInit FAILED! ret = ");
      Serial.println(ret);
    }else{
      Serial.print("\tInit complete!\t");
      Serial.print(pressure);
      Serial.println(" Pascal");
    }
    j++;
  }

  pixels.begin(); // This initializes the NeoPixel library.

  pinMode(LED_PIN, OUTPUT);
  
  for(int sensor = 0; sensor<8; sensor++){
    pressure_mean[sensor] = measure(sensor);
  }
  // intialize mean for 5 seconds
  unsigned long t0 = millis(), t1;
//  for(int sensor = 0; sensor<8; sensor++){
//    pixels.setPixelColor(PIXEL(sensor), pixels.Color(255,255,255,255));
//  }
//  pixels.show();
//  while(true){
//    
//  }
  do{
    t1 = millis();
    for(int sensor = 0; sensor<8; sensor++){
      int32_t pressure = measure(sensor);
      pressure_mean[sensor] = 0.5*pressure_mean[sensor] + 0.5*pressure;
      double counter = (t1-t0)/3000.0;
      pixels.setPixelColor(PIXEL(sensor), pixels.Color((255-(1.0-counter)*255),0,0,0));
    }
    pixels.show();
    delay(1);
  }while((t1-t0)<3000);
}

void loop()
{
  digitalWrite(LED_PIN, HIGH);
	for(int sensor = 0; sensor<8; sensor++){
    int32_t pressure = measure(sensor);
    pressure_mean[sensor] = 0.99*pressure_mean[sensor] + 0.01*pressure;
    int32_t diff = pressure - pressure_mean[sensor];
    if(diff < -SENSITIVITY){
      uint8_t white = min((-diff-DEADBAND)*SENSITIVITY,255);
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,0,white));
    }else if(diff>=-SENSITIVITY && diff<=DEADBAND) {
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
// delay(10);
 digitalWrite(LED_PIN, LOW);
}
