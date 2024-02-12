typedef struct Sensor{
  byte StationID;
  byte LeafTempID;
  int16_t LeafTempValue;
  byte LeafHum1ID;
  int16_t LeafHum1Value;
  byte LeafHum2ID;
  int16_t LeafHum2Value;
};

Sensor sensorData;

void initSensorData(Sensor *pp_sensor){
  pp_sensor->StationID = 1;
  pp_sensor->LeafTempID = 0;
  pp_sensor->LeafHum1ID = 1;
  pp_sensor->LeafHum2ID = 2;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initSensorData( &sensorData );
  Serial.println(sensorData.StationID);
  Serial.println(sensorData.LeafHum2ID);
}

void loop() {
  // put your main code here, to run repeatedly:

}
