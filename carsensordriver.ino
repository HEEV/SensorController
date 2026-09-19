#include <DS18B20.h>
#include <stdint.h>
#include <string.h>

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

/* Output pins, named so the packet's output channels and the pinMode calls
   below cannot drift apart. */
#define RAD_FAN_PIN    10
#define WATER_PUMP_PIN 11
#define SPARE_OUT_PIN  12

/* Wraps at 65535; the receiver counts gaps in it as dropped packets. */
static uint16_t sequence = 0;

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
  /* One buffered write rather than four: header, version, length, payload,
     and checksum are all assembled by the shared encoder, so the Pi's
     decoder and this cannot disagree about the format. */
  uint8_t frame[SH_FRAME_SIZE];
  size_t written = 0;

  if (sh_encode_frame(&packet, frame, sizeof(frame), &written) != SH_OK) {
    return;  /* cannot happen with a correctly sized buffer, but do not
                transmit a half-built frame if it ever does */
  }

  Serial.write(frame, written);
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
  pinMode(RAD_FAN_PIN, OUTPUT);    // Rad Fan Signal Out
  pinMode(WATER_PUMP_PIN, OUTPUT); // Water Pump Signal Out
  pinMode(SPARE_OUT_PIN, OUTPUT);
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

  DataPacket packet;
  memset(&packet, 0, sizeof(packet));

  packet.speed = speed;
  packet.airspeed = (float)analogRead(A2) - airOffset;

  packet.temps[SH_TEMP_ENGINE] = engTemp;
  packet.temps[SH_TEMP_RADIATOR] = radTemp;
  // temps[2] and temps[3] are spare. Adding a third probe to the same
  // DS18B20 bus is now a firmware change, not a wire format change.

  packet.analog[SH_ANALOG_BATTERY] = (uint16_t)analogRead(A7);
  // analog[1..3] spare, for the next sensor that needs an ADC pin.

  sh_set_digital_in(&packet, 0, digitalRead(4));
  sh_set_digital_in(&packet, 1, digitalRead(5));
  sh_set_digital_in(&packet, 2, digitalRead(6));
  sh_set_digital_in(&packet, 3, digitalRead(7));
  sh_set_digital_in(&packet, 4, digitalRead(8));

  // Report what we drive, not only what we read. The fan and pump pins
  // appeared nowhere in telemetry before, so there was no way to see or log
  // what the car was doing to itself.
  sh_set_digital_out(&packet, 0, digitalRead(RAD_FAN_PIN));
  sh_set_digital_out(&packet, 1, digitalRead(WATER_PUMP_PIN));
  sh_set_digital_out(&packet, 2, digitalRead(SPARE_OUT_PIN));

  // Wraps at 65535, which the receiver expects. Gaps here are the only way
  // the Pi can know a packet never arrived; a checksum cannot tell it.
  packet.sequence = sequence++;

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
