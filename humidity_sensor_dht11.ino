/*
 * Program Example for sensing humidity (relative humidity [%]) 
 * and even temperature [^oC] from DHT11 sensor at 2 second intervals.
 * Results are in integers
 * 
 * To obtain faster sensing rates, for example, use the
 * DHT22 sensor (1 second) or DS18B20 (0.1 second) sensors 
 * info from: https://forum.arduino.cc/index.php?topic=353392.0
 */

// DHT sensor library
// obtained from: http://www.circuitbasics.com/how-to-set-up-the-dht11-humidity-sensor-on-an-arduino/
//                name - DHTLib (a zip file somewhere in the middle of the link)

#include <dht.h> 

dht DHT;

#define DHT11_PIN 7 //pin could be digital or even analog

void setup()
{
  Serial.begin(9600);
  pinMode(DHT11_PIN,INPUT);
}

void loop()
{
  int chk = DHT.read11(DHT11_PIN);
  
  Serial.print("Temperature = ");
  Serial.print(DHT.temperature);
  Serial.print(" ^0C");

  Serial.println();
  
  Serial.print("Humidity = ");
  Serial.print(DHT.humidity);
  Serial.print(" %");

  Serial.println();

  delay(2000); //DHT11 can only sense a maximum of 2 second intervals
}

