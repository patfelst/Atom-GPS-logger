/*
M5Stack Atom GPS kit data log to micro SD card. Ability to configure the GPS module
via the ESP32 UART.

Developed using the PlatformIO (PIO) microcontrolller IDE. PIO is a plugin for
Visual Studio Code. The Arduino code framework is used.

This program uses the following libraries, unmodified:
FastLED
TinyGPS++

Version         Author              Date          Change
v0.1            Patrick Felstead    5-May-2021    First release
*/
#include <Arduino.h>
#include <FastLED.h>
#include <SD.h>  // This is the Espressif implemenation of SD.h. Don't install the Arduino SD.h to this project!
#include <SPI.h>
#include <TinyGPS++.h>

#define ATOM_GPS_LOGGER_VERS "0.1"

// SD card pins
#define SPI_SCK  23
#define SPI_MISO 33
#define SPI_MOSI 19
#define SPI_CS   -1

// Atom built in RGB LED
#define NUM_BUILTIN_LEDS             1
#define RGBLED_BUILTIN_PIN           27
#define RGBLED_BUILTIN_BRIGHTNESS_PC 100  // RGB LED Brightness in %
CRGB leds_builtin[NUM_BUILTIN_LEDS];      // FastLED "array" for single builtin RGB LED

// Serial1 is a hardware UART serial port on the ESP32 connected to the GPS module
#define GPSSerial Serial1

// The TinyGPS++ object - See website for documentation http://arduiniana.org/libraries/tinygpsplus/
TinyGPSPlus gps;

// Micro SD card file instantiation
File log_file;

uint32_t last_gps_second = 0;
uint32_t blink_led_time = 0;
char csv_string[150] = {0};

// Function prototypes
void flash_builting_RGB(CRGB led[NUM_BUILTIN_LEDS], uint32_t color, uint8_t brightness_percent, uint8_t freq, int16_t flash_number);
void rgb_led_to_gps_quality(CRGB led[NUM_BUILTIN_LEDS], uint8_t sats);
void ublox_disable_nmea(const char *nmea_type);
uint8_t display_raw_NMEA(uint8_t num_lines);

/*
-----------------
 Arduino Setup, runs once
-----------------
*/
void setup() {
  /*******************
   * Setup RGB LED
   * ****************/
  // Setup Atom's single built in RGB LED
  FastLED.addLeds<WS2812, RGBLED_BUILTIN_PIN, GRB>(leds_builtin, NUM_BUILTIN_LEDS);

  /*******************
   * Setup debug monitor serial port
   * ****************/
  Serial.begin(115200);
  Serial.println("Atom GPS logger");
  // Give user time to connect debug serial monitor
  flash_builting_RGB(leds_builtin, CRGB::DarkBlue, 100, 5, 25);

  /*******************
   * Setup Atom GPS module
   * Note: Had to wire in the GPS RX pin from 4-pin header to ESP32 Tx pin 21 inside the Atom
   * ****************/
  GPSSerial.begin(9600, SERIAL_8N1, 22, 21);
  // display_raw_NMEA(8);
  ublox_disable_nmea("GLL");
  ublox_disable_nmea("GSV");
  ublox_disable_nmea("GSA");
  ublox_disable_nmea("VTG");
  // Serial.println();
  // display_raw_NMEA(8);

  /*******************
   * Initialise micro SD card
   * ****************/
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  if (!SD.begin(SPI_CS, SPI, 40000000)) {
    Serial.println("Micro SD card failed to initialise \"Err-4\"");
    do {
      flash_builting_RGB(leds_builtin, CRGB::Red, 100, 2, 4);
      FastLED.delay(1000);
    } while (true);
  }
  Serial.println("SD card initialized OK");

  char filename[15] = {0};
  uint16_t sequence_no = 1;
  sprintf(filename, "/GPS%04d.log", sequence_no);
  while (SD.exists(filename)) {
    sequence_no++;
    sprintf(filename, "/GPS%04d.log", sequence_no);
  }
  log_file = SD.open(filename, FILE_WRITE);

  if (log_file) {
    Serial.printf("Opened micro SD card filename \"%s\" OK.\n", filename);
    log_file.printf("Atom GPS logger version v%s. TinyGPS++ library v%s.\n", ATOM_GPS_LOGGER_VERS, gps.libraryVersion());
    log_file.println("ddmmyy, hh:mm:ss, Lat, Long, Alt, Speed, Course, HDOP, PFA, CF, CCP, CCF, SIV");
    log_file.flush();
  } else {
    Serial.printf("Error opening micro SD card filename \"%s\". \"Err-5\"\n", filename);
    // Flash red LED with error code 4, forever
    do {
      flash_builting_RGB(leds_builtin, CRGB::Red, 100, 2, 5);
      FastLED.delay(1000);
    } while (true);
  }
  Serial.println();
}

/*
-----------------
  Main Arduino loop, runs forever
-----------------
*/
void loop() {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  // Since we are processing two NMEA sentences (GGA and RMC), to avoid double samples
  // only log when gps.time.isUpdated() and "gps.time.seconds" are different to the previous
  if (gps.time.isUpdated() && last_gps_second != gps.time.second()) {
    // Everything in this if block should be executed about once per second
    last_gps_second = gps.time.second();

    // Clear the log buffer first
    memset(csv_string, 0, sizeof(csv_string));

    // Blink RGB LED @ 20% brightness every 2 seconds to indicate GPS fix quality
    if (millis() - blink_led_time >= 500) {
      blink_led_time = millis();
      rgb_led_to_gps_quality(leds_builtin, gps.satellites.value());
    }

    // Don't enforce every field to be valid before logging it, i.e. the approach is not be too restricitve.
    // Post analysis will need to check the "quality" fields (8-13) to determine data "goodness".
    // However we will check that location and date are valid as they are fundamental
    // Note: invalid or not updated values will be reported as zero. Field (9 - age) will be 2^32 = 4,294,967,296
    if (gps.location.isValid() && gps.date.isValid()) {
      // 1)  Date UTC                           ddmmyy
      // 2)  Time UTC                           hh:mm:ss
      // 3)  Latitiude (decimal deg)            n.xxxxxx
      // 4)  Long (decimal deg)                 n.xxxxxx
      // 5)  Altitude (metres)                  n.xx
      // 6)  Speed (m/s)                        n.xx
      // 7)  Course (deg)                       xxx.xx
      // 8)  HDOP (100ths)                      n.xx (Horizontal Dilution of Position / Relative accuracy of horizontal position)
      // 9)  PFA - Position Fix Age (ms)        n (Number of milliseconds since position last updated)
      // 10) CF - Cumulative Fixes              n (Cumulative number of NMEA sentences with a fix, should increment in twos - GGA and RMC)
      // 11) CCP - Cumulative Checksum Passes   n (Cumulative number of NMEA sentences with a PASSED checksum, should be equal to (field 10))
      // 12) CCF - Cumulative Checksum Fails    n (Cumulative number of NMEA sentences with a FAILED checksum)
      // 13) SIV - Num Satellites In View       n (0 - 14)
      //
      //              ddmmyy,   hh:mm:ss,      Lat,  Long, Alt, Speed, Course, HDOP, PFA,CF, CCP,CCF,SIV
      //                   1    2              3     4     5    6      7       8     9   10  11  12  13
      sprintf(csv_string, "%d, %02d:%02d:%02d, %.6f, %.6f, %.2f, %.2f, %06.2f, %.2f, %d, %d, %d, %d, %d",
              gps.date.value(),                                       // (1)
              gps.time.hour(), gps.time.minute(), gps.time.second(),  // (2)
              gps.location.lat(),                                     // (3)
              gps.location.lng(),                                     // (4)
              gps.altitude.meters(),                                  // (5)
              gps.speed.mps(),                                        // (6)
              gps.course.deg(),                                       // (7)
              gps.hdop.hdop(),                                        // (8)
              gps.location.age(),                                     // (9)
              gps.sentencesWithFix(),                                 // (10)
              gps.passedChecksum(),                                   // (11)
              gps.failedChecksum(),                                   // (12)
              gps.satellites.value());                                // (13)
    } else {
      sprintf(csv_string, "Location: %s, Date: %s",
              gps.location.isValid() ? "Valid" : "Invalid (No GPS fix)",
              gps.date.isValid() ? "Valid" : "Invalid");
    }

    // If a PC is connected, display for development / debugging
    Serial.println(csv_string);

    // log the data to the SD card buffer. The buffer is 512 bytes, but we won't risk filling it, call flush() straight away
    log_file.println(csv_string);
    // Force the SD card buffer to be written to the SD card
    log_file.flush();

    // We can afford to delay 1/10th sec to keep the RGB LED lit longer as the 1 second NMEA update has only just started
    // Execution time from just after .isUpdated() to here is about 20 ms
    delay(100);
    leds_builtin[0] = CRGB::Black;
    FastLED.show();
  }
}

/*
-----------------
  Send command to UBLOX GPS to disable an NMEA sentence
-----------------
*/
void ublox_disable_nmea(const char *nmea_type) {
  char msg[30] = {0};
  char cmd[40] = {0};

  sprintf(msg, "PUBX,40,%s,0,0,0,0,0,0", nmea_type);
  // find checksum
  char checksum = 0;
  for (uint8_t i = 0; msg[i]; i++)
    checksum ^= (char)msg[i];

  // create command string
  sprintf(cmd, "$%s*%2X", msg, checksum);
  GPSSerial.println(cmd);
  Serial.println(cmd);
}

/*
-----------------
  Flash the RGB LED on the PCB
  color - colour to flash
  brightness_percent - percentage brightness
  freq - freq in Hz to flash
  flash_number - number of flashes to do. -1 = forever
-----------------
*/
void flash_builting_RGB(CRGB led[NUM_BUILTIN_LEDS], uint32_t color, uint8_t brightness_percent, uint8_t freq, int16_t flash_number) {
  uint32_t period = 500 / freq;  // Double the frequency of 1000ms period
  uint32_t count = flash_number;
  uint8_t led_bright_pc = ((brightness_percent * 0xFF) / 100);

  FastLED.setBrightness(led_bright_pc);

  while (count > 0) {
    led[0] = color;
    FastLED.show();
    delay(period);
    led[0] = CRGB::Black;
    FastLED.show();
    delay(period);
    count--;
  };
}

/*
-----------------
  Set RGB LED to display colour based on quality of GPS fix using satellites in view
  BAD / OK / GOOD = RED/AMBER/GREEN
-----------------
*/
void rgb_led_to_gps_quality(CRGB led[NUM_BUILTIN_LEDS], uint8_t sats) {
  if (sats < 4)
    led[0] = CRGB::Red;
  else if (sats >= 4 && sats <= 8)
    led[0] = CRGB::Orange;
  else if (sats > 8)
    led[0] = CRGB::Green;
  else
    led[0] = CRGB::Pink;  // Should never see this

  FastLED.show();
}

/*
-----------------
  Read and display raw NMEA sentences for the "duration" in ms
  Starts displaying anywhere in the sentence, and finishes at the end of a sentence after the duration
  Times out after 3 seconds if no characters are received from the GPS module
-----------------
*/
uint8_t display_raw_NMEA(uint8_t num_lines) {
  char c = 0;
  uint8_t line_counter = 0;
  bool timed_out = false;
  uint32_t timeout = millis() + 3000;

  do {
    if (GPSSerial.available()) {
      c = GPSSerial.read();
      Serial.printf("%c", c);
      if (c == '\n') {
        line_counter++;
        timeout = millis() + 3000;
      }
    } else if (millis() > timeout)
      timed_out = true;
  } while (!timed_out && line_counter < num_lines);

  if (timed_out) {
    Serial.println("Timed out waiting for GPS data.");
    return 1;  // timed out
  } else
    return 0;  // all good
}