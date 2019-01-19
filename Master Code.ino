/* Defining PIN assignment
  // Voltage Sensor circuit
      A1 - Voltage Divider

  // Current Sensor circuit
      A0 - Hall Effect sensor

  // Temperature Sensor circuit
      A2 - Temperature sensor

  // SD Card Module SPI bus circuit
      pin 11 - MOSI
      pin 12 - MISO
      pin 13 - CLK
      pin 4 - CS (depends on your SD card shield or module, 4 used here)

  // Relay circuit
      pin 6 - Relay PWM pin

  // LED Test Interface
      pin 2 - Arduino ON/OFF
      pin 3 - Voltage reading
      pin 5 - Current reading
      pin X - Relay Open/Close

  // DS3231 attached to Arduino as follows:
      GND - GND on Arduino
      VCC - 5V on Arduino
      SDA - A4 on Arduino
      SCL - A5 on Arduino
*/


// Header files
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "DS3231.h"

RTClib RTC;


// Declare all global variables
  // Volatge Sensor
    double DivVoltage = 0;       // Voltage divider reading
    double SourceVoltage = 0;        // Final voltage reading result

  // Current Sensor
    double HallValue1 = 0;     // Variables for current running average
    double HallValue2 = 0;
    double HallValue3 = 0;
    double HallVoltage = 0;   // Voltage reading of Hall Effect
    double HallAmps = 0;      // Current result from Hall Effect Sensor

  // Temperature Sensor
    double TempVolt = 0; //previously volt
    double Temp = 0;      // previously temp

  // Data Logging
    File myFile;
    unsigned long Time;

  // Relay Circuit
    int RelayTest;


// Declare any global constants
double RH = 969000;   // Voltage Divider High Resistance
double RL = 24783;    // Voltage Divider Low Resistance


// Setup Function
void setup() {
  Serial.begin(9600);             // Setup Baud rate
  Wire.begin();

  // Setup for Voltage Sensor
  HallValue1 = analogRead(A0);     // Take one initial voltage reading
  HallValue2 = analogRead(A0);     // Take second initial voltage reading

  // Data Logging setup
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }

  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  // Defining LED Interface pins
  pinMode(2,OUTPUT);    // Arduino On LED
  pinMode(3,OUTPUT);    // Voltage LED
  pinMode(5,OUTPUT);    // Current LED
  pinMode(6,OUTPUT);    // Relay Open/Close
}


void loop() {
  // will change after adding timer circuit

  DateTime now = RTC.now();

  /* Current Sensor */
  HallEffect();

  /* Voltage Divider */
  VoltageDivider();

  /* Temperature Sensor */
  Thermolcouple();

  /* LED Interface Control Testing */
  LEDInterface();

  /* Displaying Sensor Results */
  DisplayResults();

  /* Data Logging */
  SDLog();

  // 1 second delay?? - make saving to SD avg every 10s - + threshhold count
  delay(1000);
}

// Voltage Divider Sensor
void VoltageDivider() {
  // Read Voltage at divider and convert to decimal
  DivVoltage = ((analogRead(A1)) / 1023.0) * 5;

  // Final Source Voltage reading
  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
void HallEffect() {
  HallValue3 = HallValue2;         // Take reading at time index (n-2)
  HallValue2 = HallValue1;         // Take reading at time index (n-1)
  HallValue1 = analogRead(A0);     // Take reading at time index (n)

  // Take running average
  HallVoltage = (HallValue1 + HallValue2 + HallValue3) / 3;

  // Convert to decimal
  HallVoltage = (HallVoltage / 1023.0) * 5;

  // Compute the current from the voltage reading
  // Equation: ...
  HallAmps = (HallVoltage / 0.13720695) - (2.51 / 0.13720695);
}

// Thermolcouple sensor
void Thermolcouple() {
  // Read sensor value and convert to mV
  TempVolt = (analogRead(A2) * 5000) / 1023 + 25;

  // Check if upper voltage bound
  if(TempVolt > 2500) {
    double volt = TempVolt - 2500;    // Assuming cold junction at 22 degree
    volt = volt / 123;                // OpAmp amplified 150 times
    Temp = (volt / 0.041) + 25;       // 0.0404 mv/degree in K type
  }

  // Else if lower bound
  else {
    double volt = 2500 - TempVolt;    // Assuming cold junction at 22 degree
                                      // The voltage cross thermolcouple is 1.67v)
    volt = volt / 123;                // OpAmp apmplified 150 times
    Temp = 25 - (volt / 0.0404);      // 0.0404 mv/degree in K type
  }
}

// Display Results
void DisplayResults() {
  // Accurate time from DS3231 timer
  Serial.print("Time = " );
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);

  // Voltage divider voltage
  Serial.print("Divider Node Voltage = ");
  Serial.print(DivVoltage,3);

  // Panel voltage Result
  Serial.print("\t Panel Voltage = ");
  Serial.print(SourceVoltage,3);

  // Hall Effect voltage reading
  Serial.print("\t Hall Effect Voltage = ");
  Serial.print(HallVoltage,3);

  // Current reading
  Serial.print("\t Panel Current = ");
  Serial.print(HallAmps,3);

  // Panel temperature
  Serial.print("\t Panel Temperature = ");
  Serial.println(Temp);
}

// LED Interface
void LEDInterface() {
  RelayTest = digitalRead(6); // Checks if relay is closed
  digitalWrite(2,HIGH);       // Arduino on light
  digitalWrite(3,LOW);        // Voltage light
  digitalWrite(5,LOW);        // Current light
  digitalWrite(6,LOW);        // Relay light

  if(SourceVoltage > 0)
    digitalWrite(3,HIGH);

  if(HallAmps > 0)
    digitalWrite(5, HIGH);

  if(RelayTest == 1)
    digitalWrite(6,HIGH);
}

// Log Results on SD
void SDLog() {
  // Open test file
  // The file name testNUM is the text file we write to
  myFile = SD.open("test158.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    // Record time
    myFile.print("Time (s) = ");
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(' ');
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.print(now.second(), DEC);

    // Record divider voltage
    myFile.print("\t Divider Node Voltage = ");
    myFile.print(DivVoltage);

    // Record panel voltage
    myFile.print("\t Panels Voltage = ");
    myFile.print(SourceVoltage);

    // Record hall effect voltage signal
    myFile.print("\t Hall Effect Voltage = ");
    myFile.print(HallVoltage);

    // Record panel current
    myFile.print("\t Panel Current = ");
    myFile.print(HallAmps);

    // Record temperature (may add another)
    myFile.print("\t Panel Temperature = ");
    myFile.println(Temp);

    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  /* \\ this code will read the data just stored on the SD Card, ensure file names match
  myFile = SD.open("test151.txt");
  if (myFile) {
    Serial.println("test151.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  */
}
