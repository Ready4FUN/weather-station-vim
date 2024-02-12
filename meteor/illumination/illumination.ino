#define LGHT_PIN PC2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(illumination());
  delay(1000);
}


String illumination(){
  int sum = 0;
  float vlght = 0;
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
  return String(vlght);
  
}
