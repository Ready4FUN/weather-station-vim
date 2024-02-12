//импорт библиотек
#include <GPRS_Shield_Arduino.h>
#include <ModbusRtu.h>
#include <SoftwareSerial.h>

//-------------GPRS---------------------------------------------//

HardwareSerial Serial1(D0, D1);
int char_; //переменая для данных приходимых с gprs

GPRS gprs(Serial1); // инициализация gprs


//-------------------------modbus rs485--------------------------------//

uint16_t au16data[16];
uint8_t u8state;
uint8_t u8query;

SoftwareSerial modbusSerial(D7, D6);

Modbus master(0,modbusSerial,4); 

modbus_t telegram[3];//список комад для modbus

unsigned long u32wait; //переменная ожидания для modbus

//------------------------CO2---------------------------------------//
SoftwareSerial co2Serial(D9, D8); // RX, TX

unsigned char hexdata[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Read the gas density command /Don't change the order


//--------------------------------датчик освещенности -----------------------//
#define LGHT_PIN A0 // аналоговый пин подклечения датчика

//-----------------------------------------sensorData-----------------------------------//
//Структура для хранения и передачи по json данных с датчиков
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
  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);
  gprs.powerOff();

  
  Serial.begin(9600);  //скорость порта
  while (!Serial) {
  }
  Serial.print("Serial init OK\r\n");

  // telegram 0: датчик температуры и влажности воздуха
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


  modbusSerial.begin(9600);
  master.start(); // baud-rate at 19200 modbus
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over

  u8state = 0;
  u8query = 0; 
  
  Serial1.begin(9600);//gprs
  
  co2Serial.begin(9600);// co2

  initSensorData( &sensorData );
  //gprs_init();
  //gprs_send(data);

  
  //delay(30000);// ожидание 3 минуты для разогрева датчика co2
}

void loop() {
  Serial.println("get rs data");
  rs485Data( &sensorData );
  Serial.println("done");
  //co2( &sensorData );
  //illumination( &sensorData );
  //getDateTime( &sensorData );
  //gprs_init();
  //gprs_send(jsonString( &sensorData ));
  //Serial.println(jsonString( &sensorData ));
  printData(&sensorData);
  delay(4000);
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
//Процедура начальной инициализации GSM модуля
void gprs_init() {  
  gprs.powerOn();
  while (!gprs.init()) {
    // если связи нет, ждём 1 секунду
    // и выводим сообщение об ошибке
    // процесс повторяется в цикле
    // пока не появится ответ от GPRS устройства
    Serial.print("GPRS Init error\r\n");
    delay(3000);
  }
  Serial.println("GPRS init success");
  
  int d = 500;
  int ATsCount = 7;
  String ATs[] = {  //массив АТ команд
    "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"",  //Установка настроек подключения
    "AT+SAPBR=3,1,\"APN\",\"internet.mts.ru\"",
    "AT+SAPBR=3,1,\"USER\",\"mts\"",
    "AT+SAPBR=3,1,\"PWD\",\"mts\"",
    "AT+SAPBR=1,1",  //Устанавливаем GPRS соединение
    "AT+HTTPINIT",  //Инициализация http сервиса
    "AT+HTTPPARA=\"CID\",1"  //Установка CID параметра для http сессии
  };
  int ATsDelays[] = {6, 1, 1, 6, 3, 3, 1}; //массив задержек
  Serial.println("GPRG Internet start");
  for(int i = 0; i < ATsCount; i++) {
    //Serial.println(ATs[i]);  //посылаем в монитор порта
    Serial1.println(ATs[i]);  //посылаем в GSM модуль
    delay(d * ATsDelays[i]);
    Serial.println(ReadGSM());  //показываем ответ от GSM модуля
    delay(d);
  }
  Serial.println("GPRG Internet complete");
}

//функция отправки данных на сервер
//data: данные для отправки
void gprs_send(String data) {  
  //отправка данных на сайт
  String httpdata = String("AT+HTTPDATA=")+ data.length() + "," + data.length()*250;
  int d = 400;
  Serial.println("Send start");
  Serial.println("setup url");
  Serial1.println("AT+HTTPPARA=\"URL\",\"http://fridriiik.pythonanywhere.com/api\"");
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial1.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial1.println(httpdata);
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial.println(data);
  Serial1.println(data);
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d * 10);
  Serial1.println("AT+HTTPACTION=1");
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d*10);
  Serial.println(ReadGSM());
  Serial.println("GET url");
  Serial1.println("AT+HTTPREAD");
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial1.println("AT+HTTPTERM");
  delay(d * 2);
  Serial.println(ReadGSM());
  delay(d);
  Serial.println("Send done");
  gprs.powerOff();
}

//считывание данных с сенсоров по modbus over rs485
//возвращает true если данные успешны считаны
//Sensor *pp_sensor - указатель на структуру для хранения данных с датчиков
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
            pp_sensor->SoilHumValue = float(au16data[5]);
            pp_sensor->SoilHumValue2 = float(au16data[6]);
            pp_sensor->SoilTempValue = float(au16data[7])/100;
            Serial.print("check data");
            return;
          }
        }
        break;
      }
  }
}

//функция считывания данных с датчика CO2
bool co2(Sensor *pp_sensor){
  co2Serial.write(hexdata,9);
  delay(500);

 for(int i=0,j=0;i<9;i++){
  if (co2Serial.available()>0){
     long hi,lo,CO2;
     int ch=co2Serial.read();

    if(i==2){ hi=ch; }   //High concentration
    if(i==3){ lo=ch; }   //Low concentration
    if(i==8) {
      CO2=hi*256+lo;  //CO2 concentration
      //Serial.print("CO2 concentration: ");
      //Serial.print(CO2);
      //Serial.println("ppm");
      if(CO2 != 0){
        pp_sensor->CO2Value = CO2;
        return true;
      } else {
        return false;
      }
    }
  }   
 }
}

//функция чтения данных с датчика освещенности
bool illumination(Sensor *pp_sensor){
  int sum, vlght = 0;
  for(int i=0;i<10;i++){
    sum += analogRead(LGHT_PIN);
    delay(100);
  }
  vlght = sum/10;
  if(vlght < 70){
    vlght = 0;
  } 
  else {
    vlght = (vlght / 256) * 666;    
  }

  pp_sensor->LightValue;
  return true;
  
}

//функция чтения данных от GSM модуля
String ReadGSM() {  
  int c;
  String v;
  while (Serial1.available()) {  //сохраняем входную строку в переменную v
    c = Serial1.read();
    v += char(c);
    delay(10);
  }
  return v;
}

//функция для начального заполнения структуры базовыми значениями
void initSensorData(Sensor *pp_sensor){
  pp_sensor->StationID = 1;
  pp_sensor->LeafTempID = 0;
  pp_sensor->LeafHum1ID = 1;
  pp_sensor->LeafHum2ID = 2;
  pp_sensor->AirHumID = 3;
  pp_sensor->AirTempID = 4;
  pp_sensor->SoilHumID = 5;
  pp_sensor->SoilTempID = 6;
  pp_sensor->CO2ID = 7;
  pp_sensor->LightID = 8;
}

//функция получения времени
void getDateTime(Sensor *pp_sensor) {
    String datetime; 
    Serial1.println("AT+CCLK?");
    delay(400);
    datetime = ReadGSM();
    
    datetime = String("20") +  datetime.substring(datetime.indexOf("\"")+1, datetime.lastIndexOf("\"")-2);
    datetime.replace("/", "-");
    datetime.replace(",", "T");
    datetime.replace("+", "Z");
    pp_sensor->dateTime = datetime;
}

//функция для фолрмирования json строки
String jsonString(const Sensor *pp_sensor){
  String jsonData = String("{[") + 
  "\"StationId\":" + "\"" + pp_sensor->StationID + "\"" + 
  "\"SensorId\":" +  "\"" +  pp_sensor->LeafHum1ID + "\"" +
  "\"Date\":" + "\"" + pp_sensor->dateTime +  "\"" + 
  "\"Value\":" + "\"" + pp_sensor->LeafHumValue + "\"" +
  "},";
  return jsonData;
}
