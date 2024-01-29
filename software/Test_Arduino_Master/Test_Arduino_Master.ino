#include <Wire.h>

uint8_t read_array[4];

int     voltage;
int     current;

void setup() {
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial for output
} 

void loop() {
  
  Wire.requestFrom(48, 4);    // request 6 bytes from slave device #8

  int count=0;

  while (Wire.available()) {  // slave may send less than requested
    char c = Wire.read();     // receive a byte as character

    read_array[count] = c;
    count             = count + 1;
    
    //Serial.print(c, HEX);     // print the character
    //Serial.print(",");
  }

  //Serial.println();

  //Reconstruct the raw values from the bytes
  voltage   = (read_array[0]*16) + read_array[1];
  current   = (read_array[2]*16) + read_array[3];

  //Do some maths to get to actual voltages and currents
  //Vdd = 3V3
  //Voltage divider is 68k / 101k so multiply by 1.48529
  //Gain of current sense is 20
  //Shunt is 50mR  
  float voltage_f   = ( voltage * 3.3 * 1.48529 ) / 1024;
  float current_f   = ( current * 3.3 ) / (20 * 1024 * 0.05);  

  Serial.print  ("V=");
  Serial.print  (voltage_f);
  Serial.print  (" V\tI=");
  Serial.print  (current_f);
  Serial.println(" A");
  delay(1000);
}
