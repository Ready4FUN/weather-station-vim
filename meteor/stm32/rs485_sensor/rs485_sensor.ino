#include <ModbusRtu.h>
#include <SoftwareSerial.h>


//-------------------------modbus rs485--------------------------------//
// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
uint8_t u8query;

SoftwareSerial mySerial(PD2, PC12);//Create a SoftwareSerial object so that we can use software serial. Search "software serial" on Arduino.cc to find out more details.

Modbus master(0, mySerial, 4); // this is master and RS-232 or USB-FTDI via software serial

/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram[3];

unsigned long u32wait;

//-----------------------------------------sensorData-----------------------------------//
typedef struct Sensor{
  byte StationID;
  String dateTime; 
  byte LeafTempID;
  float LeafTempValue;
  byte LeafHum1ID;
  int16_t LeafHumValue;
  byte LeafHum2ID;
  int16_t LeafHumValue2;
  byte AirHumID;
  float AirHumValue;
  byte AirTempID;
  float AirTempValue;
  byte SoilHumID;
  float SoilHumValue;
  float SoilHumValue2;
  byte SoilTempID;
  float SoilTempValue;
  byte CO2ID;
  int16_t CO2Value;
  byte LightID;
  float LightValue;
};

//глобальная инициилизация структуры. Возможно стоит перенсти
Sensor sensorData;

//строка для отправки
String jsonData = "";


void setup() {
  Serial.begin(9600);

  telegram[0].u8id = 2; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 2; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: датчик температуры и влажности листа
  telegram[1].u8id = 3; // slave address
  telegram[1].u8fct = 3; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 0; // start address in slave
  telegram[1].u16CoilsNo = 3; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+2; // pointer to a memory array in the Arduino

  // telegram 2: датчик температуры и влажности почвы
  telegram[2].u8id = 4; // slave address
  telegram[2].u8fct = 3; // function code (this one is write a single register)
  telegram[2].u16RegAdd = 30; // start address in slave
  telegram[2].u16CoilsNo = 3; // number of elements (coils or registers) to read
  telegram[2].au16reg = au16data+5; // pointer to a memory array in the Arduino
  
  mySerial.begin(9600);//use the hardware serial if you want to connect to your computer via usb cable, etc.
  master.start(); // start the ModBus object.
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u8state = 0;
  u8query = 0;
}

void loop() {
  rs485Data(&sensorData);
  printData(&sensorData);
}

void rs485Data(Sensor *pp_sensor){
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
          u32wait = millis() + 3000;
    
          Serial.println("idle data");
          Serial.println(au16data[0]);
          Serial.println(au16data[2]);
          if(au16data[0] && au16data[2]){//проверка что данные пришли
            pp_sensor->AirHumValue = float(au16data[0])/10;
            pp_sensor->AirTempValue = float(au16data[1])/10;
            pp_sensor->LeafTempValue = float(au16data[2])/100;
            pp_sensor->LeafHumValue = au16data[3];
            pp_sensor->LeafHumValue2 = au16data[4];
            pp_sensor->SoilHumValue = au16data[5];
            pp_sensor->SoilHumValue2 = au16data[6];
            pp_sensor->SoilTempValue = float(au16data[7])/100;
            Serial.print("check data");
            return;
          }
        }
        break;
      }
  }
}

void printData(Sensor *pp_sensor){
   Serial.print("Air hum = ");
   Serial.println(pp_sensor->AirHumValue);

   Serial.print("Air temp = ");
   Serial.println(pp_sensor->AirTempValue);

   Serial.print("Leaf temp = ");
   Serial.println(pp_sensor->LeafTempValue);

   Serial.print("Lead hum = ");
   Serial.println(pp_sensor->LeafHumValue);

   Serial.print("Leaf hum 2 = ");
   Serial.println(pp_sensor->LeafHumValue2);
   
   Serial.print("Soil hum = ");
   Serial.println(pp_sensor->SoilHumValue);

   Serial.print("Soil hum 2 = ");
   Serial.println(pp_sensor->SoilHumValue2);

   Serial.print("Soil temp = ");
   Serial.println(pp_sensor->SoilTempValue);  
}
  
