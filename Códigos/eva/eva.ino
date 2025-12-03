#include <SPI.h>
#include <SD.h>
#include "Otimizador.h"
#include "Pso.h"
#include "Custos.h"

// --- PINAGEM DO HARDWARE ---
const int PIN_ESQ_PWM = 5;
const int PIN_ESQ_GND = 3;
const int PIN_DIR_PWM = 9;
const int PIN_DIR_GND = 6;
const int PIN_LED     = 7;
const int PIN_SENSOR  = A0;

// --- CORREÇÃO APLICADA AQUI ---
const int PIN_CS_SD   = 4; // Confirmado: Seu CS é o 4!

// --- CONFIGURAÇÕES DO EXPERIMENTO ---
const unsigned long TEMPO_DE_EXECUCAO_MS = 5000; // 5 segundos de teste por partícula
const int SETPOINT_DISTANCIA = 30;               // Queremos manter 30cm
const int VELOCIDADE_BASE = 65;                  // Velocidade da roda direita (Fixa)

// --- ESTADOS DA MÁQUINA ---
enum Estado {
  BOOT,           // Inicialização
  CONTAGEM,       // 10s para reposicionar (Pisca LED)
  EXECUCAO,       // Robô andando (PID ativo)
  AVALIACAO,      // Cálculo da nota
  SALVAMENTO      // Gravação no SD
};

Estado estadoAtual = BOOT;
unsigned long tempoInicioEstado = 0;

// Objetos Globais
Otimizador* otimizador = nullptr;
FuncaoCusto* custo = nullptr;

// Variáveis de Controle
float Kp = 0, Ki = 0, Kd = 0;
float erroAnterior = 0, integralErro = 0;
float dist = 0, erro = 0, pid_out = 0;

// --- FUNÇÕES AUXILIARES ---

float lerDistancia() {
  int leitura = analogRead(PIN_SENSOR);
  // Sua equação calibrada
  float cm = (0.0005 * leitura * leitura) - (0.3933 * leitura) + 88.718;
  if (cm < 10) cm = 10;
  if (cm > 90) cm = 90;
  return cm;
}

void pararMotores() {
  analogWrite(PIN_ESQ_PWM, 0);
  analogWrite(PIN_DIR_PWM, 0);
  digitalWrite(PIN_LED, LOW);
}

void acionarMotores(float controlePID) {
  analogWrite(PIN_DIR_PWM, VELOCIDADE_BASE); // Roda Direita Fixa
  
  int pwmEsq = VELOCIDADE_BASE + (int)controlePID;
  
  // Limites de segurança (0-255)
  if (pwmEsq > 255) pwmEsq = 255;
  if (pwmEsq < 0) pwmEsq = 0;
  
  analogWrite(PIN_ESQ_PWM, pwmEsq);
}

void piscarLed(int intervalo) {
  unsigned long t = millis();
  if ((t / intervalo) % 2 == 0) digitalWrite(PIN_LED, HIGH);
  else digitalWrite(PIN_LED, LOW);
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  // Configura Pinos
  pinMode(PIN_ESQ_PWM, OUTPUT); pinMode(PIN_ESQ_GND, OUTPUT);
  pinMode(PIN_DIR_PWM, OUTPUT); pinMode(PIN_DIR_GND, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  digitalWrite(PIN_ESQ_GND, LOW);
  digitalWrite(PIN_DIR_GND, LOW);

  // --- SEGREDO DO PINO 4 ---
  // O pino 10 TEM que ser saída para o SD no pino 4 funcionar
  pinMode(10, OUTPUT); 
  digitalWrite(10, HIGH); 

  Serial.print("Iniciando SD no pino 4... ");
  if (!SD.begin(PIN_CS_SD)) {
    Serial.println("FALHA!");
    while(1) { piscarLed(100); } // Pisca rápido se falhar
  }
  Serial.println("OK.");

  // Configura Algoritmos
  otimizador = new Pso();   // Cérebro: PSO
  custo = new CustoMSE();   // Juiz: Erro Quadrático Médio
  
  // Recupera treino anterior se houver queda de energia
  if (!otimizador->carregarEstado()) {
    otimizador->inicializar(); 
  }

  estadoAtual = CONTAGEM;
  tempoInicioEstado = millis();
}

// --- LOOP ---
void loop() {
  switch (estadoAtual) {
    
    // 1. CONTAGEM (Reposicionamento)
    case CONTAGEM:
    {
      unsigned long decorrido = millis() - tempoInicioEstado;
      
      if (decorrido < 7000) piscarLed(500);       // 0-7s: Pisca Lento (Posicione)
      else if (decorrido < 10000) piscarLed(100); // 7-10s: Pisca Rápido (Solte!)
      else {
        digitalWrite(PIN_LED, HIGH); // Aceso = Valendo!
        
        // Reset para nova rodada
        erroAnterior = 0; integralErro = 0;
        custo->reset();
        
        otimizador->getParametrosAtuais(Kp, Ki, Kd); // Pega novos Kp, Ki, Kd
        
        Serial.print("Rodando Particula... PID: ");
        Serial.print(Kp); Serial.print(" "); Serial.print(Ki); Serial.print(" "); Serial.println(Kd);
        
        estadoAtual = EXECUCAO;
        tempoInicioEstado = millis();
      }
      break;
    }

    // 2. EXECUÇÃO (O Robô Anda)
    case EXECUCAO:
    {
      if (millis() - tempoInicioEstado > TEMPO_DE_EXECUCAO_MS) {
        pararMotores();
        estadoAtual = AVALIACAO;
        break;
      }

      dist = lerDistancia();
      erro = SETPOINT_DISTANCIA - dist;
      
      // O Juiz anota o erro
      custo->acumular(erro, millis() - tempoInicioEstado);

      // --- CÁLCULO PID ---
      float P = Kp * erro;
      
      integralErro += erro;
      if(integralErro > 50) integralErro = 50; // Anti-windup
      if(integralErro < -50) integralErro = -50;
      float I = Ki * integralErro;
      
      float D = Kd * (erro - erroAnterior);
      erroAnterior = erro;
      
      pid_out = P + I + D;
      acionarMotores(pid_out);
      
      // Log para Excel (A cada 50ms para não travar o SD)
      static unsigned long ultimoLog = 0;
      if (millis() - ultimoLog > 50) {
        otimizador->salvarLog(dist, pid_out, erro);
        ultimoLog = millis();
      }
      
      delay(10); // Estabilidade
      break;
    }

    // 3. AVALIAÇÃO (Fim da Rodada)
    case AVALIACAO:
    {
      float notaFinal = custo->getCustoFinal();
      Serial.print("Nota da Rodada: "); Serial.println(notaFinal);
      
      otimizador->setErroDaRodada(notaFinal); // PSO aprende
      otimizador->proximaParticula();         // Prepara próxima
      
      if (otimizador->isConcluido()) {
        Serial.println("FIM DO TREINO!");
        while(1) piscarLed(2000);
      }
      
      estadoAtual = SALVAMENTO;
      break;
    }

    // 4. CHECKPOINT (Salva tudo)
    case SALVAMENTO:
    {
      otimizador->salvarEstado(); // Salva binário (cérebro)
      estadoAtual = CONTAGEM;     // Volta para o começo
      tempoInicioEstado = millis();
      break;
    }
  }
}