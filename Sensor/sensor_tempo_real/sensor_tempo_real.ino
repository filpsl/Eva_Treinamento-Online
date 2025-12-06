const int PIN_SENSOR  = A0;
float dist = 0.0;

float lerDistancia() {
  int leitura = analogRead(PIN_SENSOR);
  // Sua equação calibrada
  float cm = 10650.08 * pow(leitura,-0.935) - 10;
  // float cm = leitura;
  // if (cm < 5) cm = 5;
  // if (cm > 20) cm = 20;
  return cm;
}


void setup(){
  Serial.begin(115200);
}

void loop(){

  dist = lerDistancia();
  Serial.print("Distância: ");
  Serial.println(dist);
}