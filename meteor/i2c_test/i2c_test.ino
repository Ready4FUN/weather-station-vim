#include <Wire.h>

byte ADDRESS_SLAVE = 0X4A; 
byte REGISTER_XY = 0X00;
byte READ_LENGTH = 3;

void setup() 
{
 Wire.begin();
 Wire.setClock(400000); // set I2C 'full-speed'
 Serial.begin(9600);
}

void loop() 
{
   Wire.beginTransmission(ADDRESS_SLAVE);  
   Wire.write(REGISTER_XY);  // set register for read
   Wire.endTransmission(false); // false to not release the line

   Wire.requestFrom(ADDRESS_SLAVE,READ_LENGTH); // request bytes from register XY
   byte buff[READ_LENGTH];    
   Wire.readBytes(buff, READ_LENGTH);
   for (int i = 0; i < READ_LENGTH; i++) {
     Serial.println(buff[i], BIN);
     Serial.println(buff[i], HEX);
   }
   Serial.println();
   delay(1000);
}
