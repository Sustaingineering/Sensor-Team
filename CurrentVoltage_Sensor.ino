double RawValue1 = 0;     // Variable for running average of analog read
double RawValue2 = 0;
double RawValue3 = 0;
double RawValueavg = 0;   // Variable for running average
double HallVoltage = 0;   // Variable for voltage reading of Hall Effect Sensor
double HallAmps = 0;      // Variable for current reading of Hall Effect Sensor
double DivVoltage = 0;    // Variable for voltage from the voltage divider

// Setup
void setup(){ 
 Serial.begin(9600);

 RawValue1 = analogRead(A0);   // Take one initial reading
 RawValue2 = analogRead(A0);   // Take second initial reading
}

// Main function
void loop(){
 // Take previous three readings
 RawValue3 = RawValue2;         // Take reading at time index (n-2)
 RawValue2 = RawValue1;         // Take reading at time index (n-1)
 RawValue1 = analogRead(A0);    // Take reading at time index (n)

 // Take running average
 RawValueavg = (RawValue1+RawValue2+RawValue3) / 3;
 
 // Convert voltage value from binary to decimal
 HallVoltage = (RawValueavg / 1023.0) * 5;

 // Compute the current from the voltage reading
 HallAmps = HallVoltage/0.13720695 - 2.51/0.13720695;

 // Read Voltage at divider
 DivVoltage = (analogRead(A1) / 1023.0) * 5;

 // Display Raw Value
 Serial.print("\t Raw Value = " );
 Serial.print(RawValueavg); 

 // Display Vout
 Serial.print("\t Vout = ");
 Serial.print(HallVoltage,2); 

 // Display current reading
 Serial.print("\t Amps = ");
 Serial.println(HallAmps,2);
 
 delay(1000); // Delay 1 second
 
}
