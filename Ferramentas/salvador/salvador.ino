#include <SPI.h>
#include <SD.h>

const int PIN_CS_SD = 4; // Seu pino confirmado

void setup() {
  Serial.begin(115200);
  
  // Configuração obrigatória do Nano/Uno para SPI
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  if (!SD.begin(PIN_CS_SD)) {
    Serial.println("ERRO_SD_INIT");
    while (1);
  }
  Serial.println("ARDUINO_PRONTO"); // Sinal para o Python saber que ligou
}

void loop() {
  if (Serial.available() > 0) {
    // Lê o nome do arquivo pedido pelo Python
    String nomeArquivo = Serial.readStringUntil('\n');
    nomeArquivo.trim();

    if (nomeArquivo.length() > 0) {
      enviarArquivo(nomeArquivo);
    }
  }
}

void enviarArquivo(String nome) {
  if (!SD.exists(nome)) {
    Serial.println("ERRO_404"); // Arquivo não encontrado
    return;
  }

  File arquivo = SD.open(nome, FILE_READ);
  if (!arquivo) {
    Serial.println("ERRO_OPEN"); // Erro ao abrir
    return;
  }

  unsigned long tamanho = arquivo.size();

  // PROTOCOLO DE SEGURANÇA:
  // 1. Avisa que vai mandar
  // 2. Manda o tamanho exato em bytes
  Serial.print("OK ");
  Serial.println(tamanho);

  // Pequeno delay para o Python processar o header
  delay(100); 

  // Envia os bytes brutos
  byte buffer[64]; // Buffer pequeno para acelerar
  while (arquivo.available()) {
    int lidos = arquivo.read(buffer, sizeof(buffer));
    Serial.write(buffer, lidos);
  }
  
  arquivo.close();
  // Não printa mais nada aqui para não sujar o binário
}