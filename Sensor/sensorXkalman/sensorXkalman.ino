#include <SPI.h>
#include <SD.h>

// --- CLASSE KALMAN FILTER ---
class SimpleKalmanFilter {
  private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate = 0;
    float _last_estimate = 0;
    float _kalman_gain = 0;

  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) {
      _err_measure = mea_e;
      _err_estimate = est_e;
      _q = q;
    }

    float updateEstimate(float mea) {
      _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
      _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
      _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + abs(_last_estimate - _current_estimate) * _q;
      _last_estimate = _current_estimate;
      return _current_estimate;
    }
};

// --- CONFIGURAÇÕES ---
const int PIN_SENSOR = A0;
const int PIN_CS_SD  = 4;     // Seu pino CS
const char* NOME_ARQUIVO = "DADOS.CSV"; // Nome constante para facilitar

// Instancia o filtro: (IncertezaMedicao, IncertezaEstimativa, RuidoMovimento)
SimpleKalmanFilter filtroDist(2.0, 2.0, 0.03);

File logFile;

void setup() {
  Serial.begin(115200);

  // --- PREPARAÇÃO DO SD ---
  pinMode(10, OUTPUT);      
  digitalWrite(10, HIGH);
  
  Serial.print("Iniciando SD... ");
  if (!SD.begin(PIN_CS_SD)) {
    Serial.println("FALHA! Verifique as conexoes.");
    // Pisca o LED do Arduino para avisar erro se não tiver Serial aberto
    pinMode(LED_BUILTIN, OUTPUT);
    while(1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
  } 
  
  Serial.println("OK.");

  // --- LIMPEZA DO ARQUIVO ANTIGO (A MÁGICA ACONTECE AQUI) ---
  if (SD.exists(NOME_ARQUIVO)) {
    Serial.print("Arquivo antigo encontrado. Apagando... ");
    SD.remove(NOME_ARQUIVO);
    Serial.println("Feito!");
  }

  // --- CRIAÇÃO DO NOVO ARQUIVO ---
  logFile = SD.open(NOME_ARQUIVO, FILE_WRITE);
  if (logFile) {
    // Cabeçalho para o Excel entender as colunas
    logFile.println("Tempo_ms;Bruto;Kalman");
    logFile.close();
    Serial.println("Novo arquivo DADOS.CSV criado com sucesso.");
  } else {
    Serial.println("Erro ao criar arquivo!");
  }
}

void loop() {
  // 1. Leitura Bruta
  int leitura = analogRead(PIN_SENSOR);
  // Sua equação de calibração
  float cm_bruto = 10650.08 * pow(leitura, -0.935) - 10;
  
  // Clamp (Limites Físicos)
  if (cm_bruto < 5) cm_bruto = 5;
  if (cm_bruto > 80) cm_bruto = 80;

  // 2. Leitura Filtrada
  float cm_kalman = filtroDist.updateEstimate(cm_bruto);

  // 3. Exibir no Serial Plotter (Para ver em tempo real)
  // Formato: "Bruto:XX,Kalman:YY"
  Serial.print("Bruto:");
  Serial.print(cm_bruto);
  Serial.print(","); 
  Serial.print("Kalman:");
  Serial.println(cm_kalman);

  // 4. Salvar no SD
  logFile = SD.open(NOME_ARQUIVO, FILE_WRITE);
  if (logFile) {
    logFile.print(millis());
    logFile.print(";");
    logFile.print(cm_bruto); // Use ponto como separador decimal no Arduino, o Excel converte depois
    logFile.print(";");
    logFile.println(cm_kalman);
    logFile.close();
  }

  delay(50); // ~20 leituras por segundo
}