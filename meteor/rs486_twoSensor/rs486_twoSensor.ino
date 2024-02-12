#include <ModbusRtu.h>

uint16_t au16data[16]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

HardwareSerial Serial1(D0, D1);

Modbus master(0,1,2); 
/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram[3];

unsigned long u32wait;

String rs485Data(){
  u32wait = millis() + 1000;
  String returnData = "";
  while(true){
    switch( u8state ) {
      case 0: 
        if (millis() > u32wait) u8state++; // wait state
        break;
      case 1: 
        master.query( telegram[u8query] ); // send query (only once)
        u8state++;
        u8query++;
        if (u8query > 3) u8query = 0;
          break;
      case 2:
        master.poll(); // check incoming messages
        if (master.getState() == COM_IDLE) {
          u8state = 0;
          if(au16data[0] && au16data[2]){//проверка что данные пришли
              returnData = String("\"airHumidity\": ") + double(au16data[0])/10 + ", \"airTemp\": " + double(au16data[1])/10 
              + ", \"leafTemp\": " + double(au16data[2])/100 + ", \"leafHumidity1\": " + au16data[3] + ", \"leafHumidity2\": " + au16data[4]
              + ", \"soilHumidity\": " + double(au16data[5])/10 + ", \"soilTemp\": " + double(au16data[6])/10;
              return returnData;
          }
          //Serial.println(au16data[0]);
          //Serial.println(au16data[1]);
          //Serial.println(au16data[2]);
          //Serial.println(au16data[3]);
          //Serial.println(au16data[4]);
          //Serial.println(au16data[5]);
          //Serial.println(au16data[6]);
          u32wait = millis() + 1000; 
        }
        break;
    }
  }  
}

void setup() {
  Serial.begin(9600);
  // telegram 0: read registers
  telegram[0].u8id = 2; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: write a single register
  telegram[1].u8id = 3; // slave address
  telegram[1].u8fct = 3; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 0; // start address in slave
  telegram[1].u16CoilsNo = 3; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+2; // pointer to a memory array in the Arduino

  // telegram 1: write a single register
  telegram[2].u8id = 4; // slave address
  telegram[2].u8fct = 3; // function code (this one is write a single register)
  telegram[2].u16RegAdd = 2; // start address in slave
  telegram[2].u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram[2].au16reg = au16data+5; // pointer to a memory array in the Arduino
	
  master.begin( 9600 ); // baud-rate at 19200
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
 // u32wait = millis() + 1000;
  u8state = u8query = 0;

   Serial.println("Start");
}

void loop() {
  Serial.println(rs485Data());
  delay(5000);
  Serial.println("CHECK");
  
}
