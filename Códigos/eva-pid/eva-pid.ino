#include "Kalman.h"

// --- PINAGEM DO HARDWARE ---
const int PIN_ESQ_PWM = 5;
const int PIN_ESQ_GND = 3;
const int PIN_DIR_PWM = 9;
const int PIN_DIR_GND = 6;
const int PIN_LED     = 7;
const int PIN_SENSOR  = A0;

// --- CONSTANTES DO ROBÔ --- 
const int SETPOINT_DISTANCIA = 65;
const int VELOCIDADE_BASE = 125;                

// --- Inicialização do Filtro de Kalman
SimpleKalmanFilter filtroDist(4.0, 2.0, 0.3);

// Variáveis de Controle
float Kp = 4.13, Ki = 0.2, Kd = 0.49;
float erroAnterior = 0, integralErro = 0;
float dist = 0, erro = 0, pid_out = 0;

// --- FUNÇÕES AUXILIARES --- 
float lerDistancia() {
  int leitura = analogRead(PIN_SENSOR);
  
  // 1. PROTEÇÃO MATEMÁTICA (Do código novo/sugestão anterior)
  if (leitura <= 0) return 90; 

  // Sua equação calibrada
  float cm = 10650.08 * pow(leitura,-0.935) - 10;
  
  if (cm < 40) cm = 40;
  if (cm > 90) cm = 90;

  float cm_filtrado = filtroDist.updateEstimate(cm);
  return cm_filtrado;
}

void acionarMotores(float controlePID) {
  // Roda Direita (Fixa)
  analogWrite(PIN_DIR_PWM, VELOCIDADE_BASE);
  
  // Roda Esquerda (Controlada)
  int pwmEsq = VELOCIDADE_BASE + (int)controlePID;

  // Limites de segurança (0-255)
  if (pwmEsq > 255) pwmEsq = 255;
  if (pwmEsq < 0) pwmEsq = 0;

  analogWrite(PIN_ESQ_PWM, pwmEsq);
}

void setup(){
  // 2. CORREÇÃO: Serial no lugar certo e velocidade padrão
  Serial.begin(115200);

  // 3. A CORREÇÃO CRUCIAL (Copiada do seu código novo)
  // Sem isso, os motores flutuam e travam o Arduino
  pinMode(PIN_ESQ_PWM, OUTPUT);
  pinMode(PIN_ESQ_GND, OUTPUT);
  pinMode(PIN_DIR_PWM, OUTPUT);
  pinMode(PIN_DIR_GND, OUTPUT);
  
  // Cria o "Terra" virtual para os motores
  digitalWrite(PIN_ESQ_GND, LOW);
  digitalWrite(PIN_DIR_GND, LOW);
  
  Serial.println("EVA PID Simples Iniciado");
}

void loop(){
  // -- CÁLCULO DO ERRO
  dist = lerDistancia();
  erro = SETPOINT_DISTANCIA - dist;

  // --- CÁLCULO PID ---
  float P = Kp * erro;
  
  integralErro += erro;
  // Anti-windup
  if(integralErro > 50) integralErro = 50; 
  if(integralErro < -50) integralErro = -50;
  
  float I = Ki * integralErro;
  float D = Kd * (erro - erroAnterior);
  
  erroAnterior = erro;
  pid_out = P + I + D;

  // Debug limpo
  Serial.print("Dist: ");
  Serial.print(dist);
  Serial.print(" | PID: ");
  Serial.println(pid_out);

  acionarMotores(pid_out);
  
  delay(10); // Estabilidade do loop
}