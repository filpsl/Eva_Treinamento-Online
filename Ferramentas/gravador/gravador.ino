#include <SPI.h>
#include <SD.h>

// --- CONFIGURAÇÃO ---
const int PIN_CS_SD = 4; // Seu pino confirmado

void setup() {
  Serial.begin(115200);
  
  // Configuração obrigatória do Nano/Uno para SPI (Pino 10 como saída)
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  Serial.println("INICIANDO_SD...");

  if (!SD.begin(PIN_CS_SD)) {
    Serial.println("ERRO_SD_INIT");
    while (1); // Trava se não houver cartão
  }
  
  Serial.println("GRAVADOR_PRONTO"); // O Python espera por essa frase
}

void loop() {
  // O Loop fica vazio, apenas esperando um comando específico
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    // Protocolo: "GRAVAR nome_do_arquivo.ext"
    if (comando.startsWith("GRAVAR ")) {
      String nomeArquivo = comando.substring(7); // Pega o nome depois do espaço
      receberArquivo(nomeArquivo);
    }
  }
}

void receberArquivo(String nome) {
  // 1. Limpeza: Remove o arquivo antigo se ele já existir
  if (SD.exists(nome)) {
    SD.remove(nome);
  }

  // 2. Criação: Abre arquivo novo para escrita
  File arquivo = SD.open(nome, FILE_WRITE);
  if (!arquivo) {
    Serial.println("ERRO_CRIAR_ARQUIVO");
    return;
  }

  // 3. Handshake 1: Avisa o PC que pode mandar o tamanho
  Serial.println("AGUARDANDO_TAMANHO");
  
  // Timeout simples para não travar eternamente
  unsigned long t_inicio = millis();
  while (Serial.available() == 0) {
    if (millis() - t_inicio > 5000) { // 5 segundos de espera
      arquivo.close();
      Serial.println("ERRO_TIMEOUT_TAMANHO");
      return;
    }
  }

  // 4. Lê o tamanho do arquivo
  String strTamanho = Serial.readStringUntil('\n');
  long tamanhoTotal = strTamanho.toInt();

  if (tamanhoTotal <= 0) {
    Serial.println("ERRO_TAMANHO_INVALIDO");
    arquivo.close();
    return;
  }

  // 5. Handshake 2: Confirma que entendeu o tamanho e pede os dados
  Serial.println("OK_MANDE_DADOS");

  // 6. Loop de Gravação (Crítico)
  long bytesGravados = 0;
  byte buffer[64]; // Buffer pequeno para casar com a Serial do Arduino
  t_inicio = millis(); // Reseta timeout para os dados

  while (bytesGravados < tamanhoTotal) {
    if (Serial.available() > 0) {
      // Lê o que tiver no buffer, até o máximo de 64 bytes por vez
      int lidos = Serial.readBytes(buffer, min(Serial.available(), 64));
      
      // Escreve no SD
      arquivo.write(buffer, lidos);
      
      bytesGravados += lidos;
      t_inicio = millis(); // Reseta o timeout a cada byte recebido com sucesso
    } else {
      // Se ficar 3 segundos sem receber nada, assume que o cabo desconectou ou PC travou
      if (millis() - t_inicio > 3000) {
        Serial.println("ERRO_CONEXAO_PERDIDA");
        arquivo.close();
        return;
      }
    }
  }

  // 7. Finalização
  arquivo.close();
  Serial.println("SUCESSO_GRAVACAO_CONCLUIDA");
}