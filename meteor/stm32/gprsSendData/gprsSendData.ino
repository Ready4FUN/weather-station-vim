#include <GPRS_Shield_Arduino.h>

HardwareSerial Serial1(D0, D1);

int char_; //переменая для данных приходимых с gprs

GPRS gprs(Serial1); // инициализация gprs

//строка для отправки
String jsonData = "{\"leafTemp\": \"21.97\", \"leafHumidity1\": \"0\", \"leafHumidity2\": \"573\", \"airHumidity\": \"32.70\", \"airTemp\": \"20.70\", \"soilHumidity\": \"0.00\", \"soilTemp\": \"0.00\", \"co2\": \"1394\"}";

void setup() {
  pinMode(D2, INPUT);
  // put your setup code here, to run once:
  gprs.powerOff();

  Serial1.begin(9600);
  Serial.begin(9600);  //скорость порта
  while (!Serial) {
  }
  Serial.print("Serial init OK\r\n");

  gprs_init();
  gprs_send(jsonData);
}

void loop() {
  // put your main code here, to run repeatedly:

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
