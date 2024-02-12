/***************************************************
* Infrared CO2 Sensor 0-50000ppm(Wide Range)
* ****************************************************
* The follow example is used to detect CO2 concentration.
 
* @author lg.gang(lg.gang@qq.com)
* @version  V1.0
* @date  2016-6-6
 
* GNU Lesser General Public License.
* See <http://www.gnu.org/licenses/> for details.
* All above must be included in any redistribution
* ****************************************************/
HardwareSerial Serial2(PC11, PC10); // RX, TX
unsigned char hexdata[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Read the gas density command /Don't change the order
void setup() {
 
  Serial.begin(9600);
  while (!Serial) {

  }
  Serial2.begin(9600);
  Serial.println("Start");
}

void loop() {
  co2();
}

void co2(){
  Serial2.write(hexdata,9);
  delay(500);
  
  for(int i=0;i<9;i++){
    if (Serial2.available()>0){
      long hi,lo,CO2;
      int ch=Serial2.read();
      
      if(i==2){     hi=ch;   }   //High concentration
      if(i==3){     lo=ch;   }   //Low concentration
      if(i==8) {
        CO2=hi*256+lo;  //CO2 concentration  
        Serial.println(CO2);

        
     }
    } // if   
  }// for
}
