#include <DS18B20.h>
#include <stdint.h>
#include <string.h>
#include <math.h>  /* NAN for a sensor that has never answered */

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

/*
 * One magnet on the wheel means one sample per revolution, so the sensor
 * updates at about 10 Hz flat out and much slower everywhere else:
 *
 *    10 Hz  ->  35.7 mph     one sample every  100 ms
 *     2 Hz  ->   7.1 mph     one sample every  500 ms
 *   0.5 Hz  ->   1.8 mph     one sample every 2000 ms
 *
 * Two consequences. Speed is a whole-revolution average, not a 50 ms one, so
 * it cannot resolve a fast transient. And the loop sends at 20 Hz, so at best
 * every other packet repeats the previous speed.
 *
 * That is also why nothing in loop() may block for longer than one pulse
 * interval: a missed edge doubles the apparent pulse period and halves the
 * reported speed.
 */
#define SPEED_STALE_MS 3800UL  /* below ~0.94 mph, report a standstill */

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

/*
 * Temperature polling: one sensor per cycle, at most every 100 ms.
 *
 * DS18B20::getTempF() is expensive in a way the call site does not show. It
 * issues CONVERT_T and then calls delayForConversion(), a blocking delay of
 * 750 ms at the library's default 12-bit resolution. Reading both sensors
 * inline every loop meant up to 1500 ms inside an iteration meant to turn
 * over in 50 ms, which is roughly 30 packets not sent.
 *
 * It does NOT lose wheel interrupts: Arduino's delay() spins on micros() with
 * interrupts enabled, so the magnet ISR keeps timestamping throughout. The
 * damage is to packet cadence, not to speed accuracy.
 *
 * Two changes bring the cost down by about 16x:
 *
 *   - 9-bit resolution. The conversion drops from 750 ms to 94 ms, and
 *     0.5 C is far finer than anything useful about coolant temperature.
 *   - One sensor per cycle, no more than every 100 ms, which is also the
 *     fastest the part can usefully be read.
 *
 * select() is not free either. Bus reset, ROM match, nine scratchpad bytes
 * and a power-mode query, with OneWire holding interrupts off around each bit
 * it times. Calling it twice per loop put all of that on the hot path; now it
 * happens once per 100 ms.
 *
 * Going fully non-blocking needs raw OneWire rather than this library, which
 * exposes no way to start a conversion and collect it later: getTempC(),
 * getTempF() and doConversion() all wait internally, and readScratchpad() is
 * private. That is a worthwhile follow-up, not a change to make blind.
 */
#define TEMP_POLL_INTERVAL_MS 100UL  /* the part cannot do better than ~90 ms */
#define TEMP_RESOLUTION       9      /* 94 ms conversion instead of 750 ms */
#define TEMP_SENSOR_COUNT     2

static const uint8_t *const tempAddr[TEMP_SENSOR_COUNT] = {
  engineTempAddr, radTempAddr
};

/*
 * Last good reading per sensor. A sensor that has never answered reports NAN
 * rather than a stale or invented number.
 *
 * This matters: the previous code fell off the end of updateEngineTemp() and
 * updateRadiatorTemp() without returning a value on the cache-hit path, which
 * is undefined behaviour, and that path ran 50 times out of every 51. The
 * caller got whatever happened to be in the return register. A radiator
 * temperature frozen at exactly 48.4 in every CSV on the car is consistent
 * with a real early reading left sitting there and never updated again.
 */
static float tempValue[TEMP_SENSOR_COUNT] = { NAN, NAN };
static uint8_t tempIndex = 0;
static unsigned long tempTimer = 0;


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

  // Step the temperature poller: reads at most one sensor, and only every
  // 100 ms, rather than both on every pass through here.
  serviceTemps();

  float engTemp = tempValue[0];
  float radTemp = tempValue[1];

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
  unsigned long lastMagnet;
  unsigned long delta;

  /*
   * Snapshot both ISR variables with interrupts off.
   *
   * These are 32-bit on an 8-bit part, so a plain read is four separate byte
   * loads. If handleMagnet() fires between them the result is half the old
   * value and half the new one, which produces a speed that was never real.
   * At 10 Hz the window is small but it is not zero, and a torn deltaTime
   * shows up as an implausible spike rather than as an obvious fault.
   */
  noInterrupts();
  lastMagnet = magnetTimes[0];
  delta = deltaTime;
  interrupts();

  if (lastMagnet == 0) {
    return 0.0f;   /* no magnet seen since boot */
  }

  if (millis() - lastMagnet >= SPEED_STALE_MS) {
    return 0.0f;   /* stopped, or slower than about 0.94 mph */
  }

  if (delta == 0) {
    return 0.0f;   /* guard the divide; debounce should prevent this */
  }

  /*
   * current magnet setup (X is a magnet)
   *   ***********
   *  *     X     *
   *  *           *
   *  *     O     *
   *  *           *
   *  *           *
   *   ***********
   */

  /* inches per second, then inches/sec -> miles/hour */
  float inps = (pulseDist / (float)delta) * 1000.0f;
  return ((inps / 12.0f) / 5280.0f) * 3600.0f;
}

// Get Temperatures, but only every so often because these sensors are slow.

/*
 * Read one temperature sensor, at most every TEMP_POLL_INTERVAL_MS.
 *
 * Blocks for one 9-bit conversion, about 94 ms, when it does read. That is
 * the best this library allows; see the note above. A sensor that does not
 * select is left as NAN rather than reported as a plausible number.
 */
void serviceTemps() {
  unsigned long now = millis();

  if (now - tempTimer < TEMP_POLL_INTERVAL_MS) {
    return;
  }
  tempTimer = now;

  if (ds.select((uint8_t *)tempAddr[tempIndex])) {
    ds.setResolution(TEMP_RESOLUTION);
    tempValue[tempIndex] = ds.getTempF();
  } else {
    /* Not on the bus. The engine probe's address is still all zeroes, so
       this is its normal path. Report NAN, not an invented value. */
    tempValue[tempIndex] = NAN;
  }

  tempIndex = (uint8_t)((tempIndex + 1) % TEMP_SENSOR_COUNT);
}

