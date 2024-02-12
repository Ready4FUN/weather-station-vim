#include <GPRS_Shield_Arduino.h>


int char_; //переменая для данных приходимых с gprs

GPRS gprs(Serial2); // инициализация gprs


void setup(){
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    gprs.powerOff();

    Serial.begin(9600);  //скорость порта
    while (!Serial) {}
    Serial.print("Serial init OK\r\n");

    Serial2.begin(9600);
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
}

void loop(){
    Serial.println(getDateTime());
}

String getDateTime() {
    String datetime; 
    Serial2.println("AT+CCLK?");
    delay(400);
    datetime = ReadGSM();
    
    datetime = String("20") +  datetime.substring(datetime.indexOf("\"")+1, datetime.lastIndexOf("\"")-2);
    datetime.replace("/", "-");
    datetime.replace(",", "T");
    datetime.replace("+", "Z");
    return datetime;
}

String ReadGSM() {  
  int c;
  String v;
  while (Serial2.available()) {  //сохраняем входную строку в переменную v
    c = Serial2.read();
    v += char(c);
    delay(10);
  }
  return v;
}