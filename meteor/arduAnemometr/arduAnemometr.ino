#include <Wire.h>

//#define NUM_READINGS 500
int average;
unsigned long lastflash;
float RPM;
float RPM2;
float veter;

char Buff[7];

void setup() {
  Serial.begin(9600);  //открыть порт
  attachInterrupt(4,sens,RISING); //подключить прерывание на 2 пин при повышении сигнала
  pinMode(3, OUTPUT);   //3 пин как выход
  digitalWrite(3, HIGH);  //подать 5 вольт на 3 пин

  Wire.begin(9);
  Wire.onRequest(requestEvents);
}
void sens() {
  RPM=60/((float)(micros()-lastflash)/1000000);  //расчет
  lastflash=micros();  //запомнить время последнего оборота
}

void loop() {
  if ((micros()-lastflash)>1000000){ //если сигнала нет больше секунды
    RPM=0;  //считаем что RPM 0
  }
  //Serial.println(RPM);   //вывод в порт
    //задержка для стабильности
  RPM2=RPM/8;
    long sum = 0;                                  // локальная переменная sum
  for (int i = 0; i < 1000; i++) {      // согласно количеству усреднений
    sum += RPM2;                        // суммируем значения с любого датчика в переменную sum
  }
  average = sum / 1000;                  // находим среднее арифметическое, разделив сумму на число измерений
  veter = average / 60.0;
  //Serial.print("Оборотs     ");
  //Serial.println(average);
  Serial.print("Ветер    ");
  Serial.println(veter);
  delay(600);
}

void requestEvents(){
  dtostrf(veter,7,2, Buff);
  Wire.write(Buff);
}
