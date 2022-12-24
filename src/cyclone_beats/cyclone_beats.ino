/*
 * find midi device port with: 
 * aseqdump -l
 * then listen to midi device:
 * aseqdump -p 24:0
 */


#include <Wire.h>
#include "ClosedCube_TCA9546A.h"
#include <Dps310.h>
#include <Adafruit_NeoPixel.h>
#include <Control_Surface.h> // Include the Control Surface library
#define LED_PIN        13
#define PIN            8
#define NUMPIXELS      8
#define DEADBAND 0
#define SENSITIVITY 1
#define PIXEL(sensor) (7-sensor)
#define KNOB_CLK 4
#define KNOB_DT 3
#define KNOB_SW 2
#define SW_0 9
#define SW_1 10

USBMIDI_Interface midi;
//USBDebugMIDI_Interface midi{115200};

#include <Control_Surface.h>

float pressure[8] = {0};
float pressure_mean[8] = {0};

BEGIN_CS_NAMESPACE

/*
 * @brief   A class for momentary buttons and switches that send MIDI events.
 *
 * The button is debounced, and the internal pull-up resistor is enabled.
 *
 * @see     NoteButton
 * @see     Button
 */
class MyNoteButton : public MIDIOutputElement {
 public:
  /*
   * @brief   Create a new MyNoteButton object on the given pin, with the 
   *          given address and velocity.
   * 
   * @param   pin
   *          The digital input pin to read from.  
   *          The internal pull-up resistor will be enabled.
   * @param   address
   *          The MIDI address to send to.
   * @param   velocity
   *          The MIDI note velocity [0, 127].
   */
  MyNoteButton(pin_t pin, MIDIAddress address, uint8_t velocity)
    : sensor(pin), address(address), velocity(velocity) {}

 public:
  // Initialize: enable the pull-up resistor for the button
  // This method is called once by `Control_Surface.begin()`.
  void begin() final override {  }

  // Update: read the button and send MIDI messages when appropriate.
  // This method is called continuously by `Control_Surface.loop()`.
  void update() final override {
    uint8_t vel = min(pressure[sensor]-pressure_mean[sensor],255);
    if ( pressure[sensor]>pressure_mean[sensor] ) {               // if pressed
      Control_Surface.sendNoteOn(address, vel);  //   → note on
    } else if (pressure[sensor]<pressure_mean[sensor]) {         // if released
      Control_Surface.sendNoteOff(address, vel); //   → note off
    }
  }

 private:
  int sensor;
  const MIDIAddress address;
  uint8_t velocity;
};

END_CS_NAMESPACE

// Instantiate a NoteButton object
MyNoteButton midi_sensors[] {
  {0, {MIDI_Notes::C(4), CHANNEL_1},0},
  {1, {MIDI_Notes::D(4), CHANNEL_1},0},
  {2, {MIDI_Notes::E(4), CHANNEL_1},0},
  {3, {MIDI_Notes::E(4), CHANNEL_1},0},
  {4, {MIDI_Notes::G(4), CHANNEL_1},0},
  {5, {MIDI_Notes::A(4), CHANNEL_1},0},
  {6, {MIDI_Notes::A(4), CHANNEL_1},0},
  {7, {MIDI_Notes::C(5), CHANNEL_1},0}
};

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

ClosedCube_TCA9546A tca9546a;

Dps310 sensors[8];

//uint8_t pressure_count = 0, temperature_count = 0;
//float pressure[32], temperature[32];

float measure(int sensor){
  int channel = sensor/2;
  tca9546a.selectChannel(channel);
  float pressure;
  int ret = sensors[sensor].measurePressureOnce(pressure,0);
//  int16_t ret = sensors[sensor].getContResults(&temperature[0], pressure_count, &pressure[0], temperature_count);
//  if(sensor==0){
//    SerialUSB.println(pressure_count);
//    SerialUSB.println(pressure[0]);
//  }
  return pressure;
}

void setup()
{
  SerialUSB.begin(115200);
  SerialUSB.println("hello");
	
	tca9546a.begin(0x70);

  int j = 0;
  for(int i=0;i<4;i++){
    tca9546a.selectChannel(i);
    sensors[j].begin(Wire, 0x76);
//    int16_t ret = sensors[j].startMeasurePressureOnce(prs_mr, prs_osr);
    sensors[j].correctTemp();
    int ret = sensors[j].measurePressureOnce(pressure[j],0);
    SerialUSB.print("sensor \t");
    SerialUSB.print(j);
    if (ret != 0){
      SerialUSB.print("\tInit FAILED! ret = ");
      SerialUSB.println(ret);
    }else{
      SerialUSB.println("\tInit complete!\t");
    }

    j++;
    sensors[j].begin(Wire, 0x77);
//    ret = sensors[j].startMeasurePressureCont(prs_mr, prs_osr);
    sensors[j].correctTemp();
    ret = sensors[j].measurePressureOnce(pressure[j],0);
    SerialUSB.print("sensor \t");
    SerialUSB.print(j);
    if (ret != 0){
      SerialUSB.print("\tInit FAILED! ret = ");
      SerialUSB.println(ret);
    }else{
      SerialUSB.println("\tInit complete!\t");
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
      pressure[sensor] = measure(sensor);
      pressure_mean[sensor] += pressure[sensor];
      double counter = (t1-t0)/3000.0;
      pixels.setPixelColor(PIXEL(sensor), pixels.Color((255-(1.0-counter)*255),0,0,0));
    }
    pixels.show();
    delay(10);
    samples++;
  }while((t1-t0)<3000);
  for(int sensor = 0; sensor<8; sensor++){
    pressure_mean[sensor] /= (float)samples;
    SerialUSB.println(pressure_mean[sensor]);
  }
  delay(500);
  Control_Surface.begin(); // Initialize Control Surface
}

void loop()
{
	for(int sensor = 0; sensor<8; sensor++){
    pressure[sensor] = measure(sensor);
    float diff = pressure[sensor] - pressure_mean[sensor];
    if(diff < -DEADBAND){
      uint8_t white = min((-diff-DEADBAND)*SENSITIVITY,255);
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,0,white));
    }else if(diff>=-DEADBAND && diff<=DEADBAND) {
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,0,0));
    }else{
      uint8_t blue = min((diff-DEADBAND)*SENSITIVITY,255);
      pixels.setPixelColor(PIXEL(sensor), pixels.Color(0,0,blue,0));
    }
	}
  pixels.show(); // This sends the updated pixel color to the hardware.
  Control_Surface.loop(); // Update the Control Surface
}
