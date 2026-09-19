#include <DS18B20.h>
#include <stdint.h>

/*
 * The packet layout, the checksum, and the frame encoder live in the
 * SensorHub library, which the Raspberry Pi uses to decode this. Sharing one
 * definition is the whole point: the two ends cannot drift apart, because
 * there is only one of them to edit.
 *
 * Install with:
 *   arduino-cli lib install --git-url https://github.com/HEEV/SensorHub
 *
 * Only the encoder is linked here, about 90 bytes of flash more than the
 * hand-rolled version it replaced. The receiving state machine comes along in
 * the same header for whenever the Pi starts commanding the output channels
 * on pins 10, 11, and 12.
 */
#include <SensorHub.h>

/* Keep the local spelling so the call sites below read unchanged. */
typedef sh_packet_t DataPacket;

//wheel speed constants
#define wheelSpeedSensorPin 2
#define numMagnets 1
#define debounceTime 10 // in ms
#define wheelRadius 10.0 // in in
#define circumference (2 * wheelRadius * PI) // in in 
#define pulseDist (circumference / numMagnets) 

// other wheelspeed variables
volatile unsigned long magnetTimes[2] = { 0 }; // volatile modifier due to write in interrupt
volatile unsigned long deltaTime = 0;
volatile unsigned long curTime = 0;
volatile float distTraveled = 0;
float speed = 0;

// Airspeed sensor variables
float airOffset = 0.0f;  // Variable to hold the 0-speed pressure
//float airDiff;    // Difference between current air pressure and offset

// Temperature Sensor variables
DS18B20 ds(3);
const uint8_t engineTempAddr[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t radTempAddr[8]    = { 0x28, 0xD0, 0xEB, 0x87, 0x00, 0xCA, 0x26, 0x82 };

// Index 0 is engine temp, index 1 is rad temp
const int cacheTTL[] = {50, 50};
int cacheLife[] = {0, 0};

void sendPacket(const DataPacket &packet) {
  /* One buffered write rather than four: header, payload, and checksum are
     assembled by the shared encoder, so the Pi's decoder and this cannot
     disagree about the format. */
  uint8_t frame[SH_FRAME_SIZE];

  sh_encode_frame(&packet, frame);
  Serial.write(frame, sizeof(frame));
}

void setup() {
  Serial.begin(115200);  //Sets frequency value. - DO NOT CHANGE

  pinMode(wheelSpeedSensorPin, INPUT_PULLUP); // Wheelspeed, pin 2
  attachInterrupt(digitalPinToInterrupt(2), handleMagnet, FALLING); // wheelspeed interrupt
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(10, OUTPUT); // Rad Fan Signal Out
  pinMode(11, OUTPUT); // Water Pump Signal Out
  pinMode(12, OUTPUT);
  pinMode(A7, INPUT);
  pinMode(A2, INPUT);  //Attach the A2 pin on the arduino to the OUT pin on the airspeed module

  // Zero out the wind speed measurements on startup
  for (int i = 0; i < 50; i++) {
    airOffset += analogRead(A2);  //Average 50 readings to smooth out the data
  }
  airOffset /= 50;  //Divide by 50 to make it the average
}

void loop() {
  // Update speed values
  speed = getSpeed();

  // Update temperature cache values
  float engTemp = updateEngineTemp();
  float radTemp = updateRadiatorTemp();

  DataPacket packet = {
    speed,
    (float)analogRead(A2) - airOffset,
    engTemp,
    radTemp,
    (uint8_t)digitalRead(4),
    (uint8_t)digitalRead(5),
    (uint8_t)digitalRead(6),
    (uint8_t)digitalRead(7),
    (uint8_t)digitalRead(8),
    (uint16_t)analogRead(A7)
  };

  if (speed > 0.25f || speed == 0.0f) {
    sendPacket(packet);
  }

  delay(50); 
}

void handleMagnet() {
  curTime = millis();
  if (curTime - magnetTimes[0] > debounceTime) {
    magnetTimes[1] = magnetTimes[0];
    magnetTimes[0] = curTime;

    // use the retrieved magnet timings to get delta t on the pulse
    deltaTime = magnetTimes[0] - magnetTimes[1];
  }
}

float getSpeed() {

  if (millis() - magnetTimes[0] < 3800 && magnetTimes[0] != 0) {
    // Calculating our speed based on the magnet timings

    /*
    current magnet setup (X is a magnet)
      ***********
     *     X     * 
     *           *
     *     O     *
     *           *
     *           *
      ***********
    */

    // Calculate speed in inches per second
    float inps = ((circumference / numMagnets) / deltaTime) * 1000.0f;

    // convert the speed we calculated from Inches/Sec to Miles/Hr
    return ((inps / 12.0f) / 5280.0f) * 3600.0f;
  }

  return 0.0;
}

// Get Temperatures, but only every so often because these sensors are slow.

float updateEngineTemp() {
  if (ds.select(engineTempAddr)){
    if (cacheLife[0] > cacheTTL[0]) {
      cacheLife[0] = 0;
      return ds.getTempF();
    } else { 
      cacheLife[0]++; 
    }
  }
}

float updateRadiatorTemp() {
  if (ds.select(radTempAddr)){
    if (cacheLife[1] > cacheTTL[1]) {
      cacheLife[1] = 0;
      return ds.getTempF();
    }
    else { 
      cacheLife[1]++; 
    }
  }
}
