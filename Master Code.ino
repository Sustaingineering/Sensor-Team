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

  // Timer pins
      A4 - SDA
      A5 - SCL

  // LED Test Interface
      pin 2 - Arduino ON/OFF
      pin 3 - Voltage reading
      pin 5 - Current reading
      pin 7 - Relay Open/Close
*/


// Header files
#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include "DS3231.h"

RTClib RTC;
SdFat SD;

// Declare all global variables
  // Volatge Sensor
    double DivVoltage = 0;       // Voltage divider reading
    double SourceVoltage = 0;        // Final voltage reading result

  // Current Sensor
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
double RH = 24743.0;   // Voltage Divider High Resistance
double RL = 981300.0;    // Voltage Divider Low Resistance


// Setup Function
void setup() {
  Serial.begin(9600);             // Setup Baud rate
  Wire.begin();

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
  pinMode(7,OUTPUT);    // Relay Open/Close
  pinMode(6,INPUT);
}


void loop() {
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
  int x = analogRead(A1);
  DivVoltage = x * (5.0/1023.0);

  // Final Source Voltage reading
  SourceVoltage = (DivVoltage * (RH + RL)) / RL;
}

// Hall Effect sensor
void HallEffect() {
  int x = analogRead(A0);     // Take reading

  // Convert to decimal
  HallVoltage = x * (5.0/1023.0);

  // Compute the current from the voltage reading
  // Equation: ...
  HallAmps = (HallVoltage / 0.13720695) - (2.51 / 0.13720695);
}

// Thermolcouple sensor
void Thermolcouple() {
  // Read sensor value and convert to mV
  int x = analogRead(A2);
  TempVolt = x * (5000.0/1023.0) + 25;

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
  // Voltage divider voltage
  Serial.print("Divider Voltage = ");
  Serial.print(DivVoltage,3);

  // Panel voltage Result
  Serial.print("\t Panel Voltage = ");
  Serial.print(SourceVoltage,3);

  // Hall Effect voltage reading
  Serial.print("\t Hall Voltage = ");
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
  digitalWrite(7,LOW);        // Relay light

  if(SourceVoltage > 0)
    digitalWrite(3,HIGH);

  if(HallAmps > 0)
    digitalWrite(5, HIGH);

  if(RelayTest == 1)
    digitalWrite(7,HIGH);
}

// Log Results on SD
void SDLog() {
  // Open test file
  // The file name testNUM is the text file we write to
  myFile = SD.open("timertestreal2.txt", FILE_WRITE);

  // if the file opened okay, write to it
  if (myFile) {
    // Record time
     DateTime now = RTC.now();
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
    myFile.print("\t Divider Voltage = ");
    myFile.print(DivVoltage);

    // Record panel voltage
    myFile.print("\t Panels Voltage = ");
    myFile.print(SourceVoltage);

    // Record hall effect voltage signal
    myFile.print("\t Hall Voltage = ");
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
    Serial.println("error opening timertest3.txt");
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
