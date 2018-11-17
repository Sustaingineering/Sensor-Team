//Current_and_Voltage function variables
double RawValue1 = 0;     // Variable for running average of analog read
double RawValue2 = 0;
double RawValue3 = 0;
double RawValueavg = 0;   // Variable for running average
double HallVoltage = 0;   // Variable for voltage reading of Hall Effect Sensor
double HallAmps = 0;      // Variable for current reading of Hall Effect Sensor
double DivVoltage = 0;    // Variable for voltage from the voltage divider
double VSource = 0; 
double RH = 969000;
double RL = 24783;

//Temp_Sensor variables
double sensorValue = 0;
double volt = 0;
double temp = 0;
double voltage = 0;

void Current_and_Voltage(){

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
 DivVoltage = analogRead(A1);
 DivVoltage = ((DivVoltage) / 1023.0) * 5;

 VSource = (DivVoltage * (RH + RL))/RL + 0.5 ;

 // Display Raw Value
 Serial.print("Raw Value = " );
 Serial.print(RawValueavg); 

 // Display Vout
 Serial.print("\t Vout = ");
 Serial.print(HallVoltage,2); 

 // Display current reading
 Serial.print("\t Amps = ");
 Serial.print(HallAmps,2);
 Serial.print("\t Volts = ");
 Serial.print(DivVoltage,6);
 Serial.print("\t VSource = ");
 Serial.println(VSource);
    
}

void Temp_Sensor(){

  sensorValue = analogRead(A2);
  voltage = sensorValue*(5000.0/1023.0) + 25;

  if(voltage > 2500)
  {
     volt = voltage-2500;//(while cold conjuction is 22 degree 
     volt =  volt / 123; //(opamp apmplified 123 times)
     temp = (volt)/0.041 + 20  ;//( 1 degree = 0.0404 mv in K type )
  }
       
  else{
     volt = 2500-voltage;//(while cold conjuction is 22 degree 
                             //   the voltage cross thermalcouple is 1.67v) 
     volt =  volt / 123; //(opamp apmplified 150 times)
        
     temp = 20-(volt)/0.0404  ;//( 1 degree = 0.0404 mv in K type )
  
  }
  
  Serial.print("sensorValue = ");
  Serial.print(sensorValue);
  Serial.print("\ttemp = ");
  Serial.println(temp);
}



// Setup
void setup(){ 
 Serial.begin(9600);

 RawValue1 = analogRead(A0);   // Take one initial reading
 RawValue2 = analogRead(A0);   // Take second initial reading
}



// Main function
void loop(){

 Current_and_Voltage();
 Temp_Sensor();
 Serial.println(" ");
 
 
 delay(2000); // Delay 1 second
 
}
