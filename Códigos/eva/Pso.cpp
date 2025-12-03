#include "Pso.h"

Pso::Pso() {
    erro_da_rodada_atual = 0.0;
    // Estado inicial seguro
    estado.inicializado = false;
    estado.iteracao_atual = 0;
    estado.particula_atual = 0;
    estado.gbest_erro = 10000000.0; // Infinito inicial
}

float Pso::randomFloat(float min, float max) {
    return min + (float)random(0, 10000) / 10000.0 * (max - min);
}

void Pso::inicializar() {
    Serial.println("PSO: Inicializando novo enxame...");
    
    estado.iteracao_atual = 0;
    estado.particula_atual = 0;
    estado.gbest_erro = 10000000.0; // Um valor muito alto

    for (int i = 0; i < NUM_PARTICULAS; i++) {
        // 1. Posições iniciais aleatórias dentro dos limites do PID
        estado.x[i][0] = randomFloat(KP_MIN, KP_MAX); // Kp
        estado.x[i][1] = randomFloat(KI_MIN, KI_MAX); // Ki
        estado.x[i][2] = randomFloat(KD_MIN, KD_MAX); // Kd

        // 2. Pbest inicial é a própria posição inicial
        for (int d = 0; d < NUM_DIMENSOES; d++) {
            estado.pbest_pos[i][d] = estado.x[i][d];
            estado.v[i][d] = 0.0; // Velocidade inicial zero
        }
        estado.pbest_erro[i] = 10000000.0; // Erro inicial infinito
    }
    
    estado.inicializado = true;
    salvarEstado(); // Garante que o arquivo exista logo de cara
}

void Pso::getParametrosAtuais(float &kp, float &ki, float &kd) {
    int i = estado.particula_atual;
    kp = estado.x[i][0];
    ki = estado.x[i][1];
    kd = estado.x[i][2];
}

void Pso::setErroDaRodada(float erro) {
    erro_da_rodada_atual = erro;
    int i = estado.particula_atual;

    // --- Lógica PSO: Atualização de Memórias ---

    // 1. Atualiza Pbest (Melhor Pessoal)
    if (erro < estado.pbest_erro[i]) {
        estado.pbest_erro[i] = erro;
        for (int d = 0; d < NUM_DIMENSOES; d++) {
            estado.pbest_pos[i][d] = estado.x[i][d];
        }
    }

    // 2. Atualiza Gbest (Melhor Global)
    if (erro < estado.gbest_erro) {
        estado.gbest_erro = erro;
        for (int d = 0; d < NUM_DIMENSOES; d++) {
            estado.gbest_pos[d] = estado.x[i][d];
        }
        Serial.print("PSO: Novo Gbest encontrado! Erro: ");
        Serial.println(estado.gbest_erro);
    }
    
    // --- Lógica PSO: Cálculo da Nova Posição (Para a PRÓXIMA iteração) ---
    // A partícula já foi testada nesta iteração. Agora calculamos para onde ela vai na próxima.
    
    for (int d = 0; d < NUM_DIMENSOES; d++) {
        float r1 = randomFloat(0.0, 1.0);
        float r2 = randomFloat(0.0, 1.0);

        // Atualiza Velocidade
        // v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)
        estado.v[i][d] = (W * estado.v[i][d]) + 
                         (C1 * r1 * (estado.pbest_pos[i][d] - estado.x[i][d])) + 
                         (C2 * r2 * (estado.gbest_pos[d] - estado.x[i][d]));

        // Atualiza Posição
        estado.x[i][d] = estado.x[i][d] + estado.v[i][d];
    }

    // Garante que o PID não fique negativo ou exploda
    limitarPosicao(i);
}

void Pso::limitarPosicao(int i) {
    // Restrições de Kp
    if (estado.x[i][0] < KP_MIN) estado.x[i][0] = KP_MIN;
    if (estado.x[i][0] > KP_MAX) estado.x[i][0] = KP_MAX;

    // Restrições de Ki
    if (estado.x[i][1] < KI_MIN) estado.x[i][1] = KI_MIN;
    if (estado.x[i][1] > KI_MAX) estado.x[i][1] = KI_MAX;

    // Restrições de Kd
    if (estado.x[i][2] < KD_MIN) estado.x[i][2] = KD_MIN;
    if (estado.x[i][2] > KD_MAX) estado.x[i][2] = KD_MAX;
}

void Pso::proximaParticula() {
    estado.particula_atual++;

    // Se passamos da última partícula, terminamos uma iteração (geração)
    if (estado.particula_atual >= NUM_PARTICULAS) {
        estado.particula_atual = 0;
        estado.iteracao_atual++;
        Serial.print("PSO: Fim da iteração ");
        Serial.println(estado.iteracao_atual);
    }
}

bool Pso::isConcluido() {
    return (estado.iteracao_atual >= MAX_ITERACOES);
}

// --- PERSISTÊNCIA NO CARTÃO SD ---

void Pso::salvarEstado() {
    // Remove arquivo anterior para garantir escrita limpa
    if (SD.exists(arquivo_sd)) {
        SD.remove(arquivo_sd);
    }

    File arquivo = SD.open(arquivo_sd, FILE_WRITE);
    if (arquivo) {
        // Escreve a struct inteira como um bloco de bytes
        arquivo.write((uint8_t *)&estado, sizeof(estado));
        arquivo.close();
        // Serial.println("PSO: Checkpoint salvo no SD.");
    } else {
        Serial.println("PSO ERRO: Falha ao salvar no SD!");
    }
}

bool Pso::carregarEstado() {
    if (!SD.exists(arquivo_sd)) {
        Serial.println("PSO: Nenhum save encontrado. Comecando do zero.");
        return false;
    }

    File arquivo = SD.open(arquivo_sd, FILE_READ);
    if (arquivo) {
        // Lê os bytes e preenche a struct
        arquivo.read((uint8_t *)&estado, sizeof(estado));
        arquivo.close();

        // Verificação básica de integridade
        if (estado.inicializado) {
            Serial.println("PSO: Save carregado com sucesso!");
            imprimirStatus();
            return true;
        }
    }
    
    Serial.println("PSO: Save corrompido ou invalido.");
    return false;
}

void Pso::imprimirStatus() {
    Serial.print("--- STATUS PSO ---\n");
    Serial.print("Iteracao: "); Serial.print(estado.iteracao_atual);
    Serial.print(" | Particula: "); Serial.println(estado.particula_atual);
    Serial.print("Melhor Erro Global (Gbest): "); Serial.println(estado.gbest_erro);
    Serial.print("Parametros Gbest (Kp, Ki, Kd): [");
    Serial.print(estado.gbest_pos[0]); Serial.print(", ");
    Serial.print(estado.gbest_pos[1]); Serial.print(", ");
    Serial.print(estado.gbest_pos[2]); Serial.println("]");
}


void Pso::salvarLog(float distancia, float pwm, float erro) {
    File dataFile = SD.open("DADOS.txt", FILE_WRITE);

    if (dataFile) {
        // Se arquivo novo, cria cabeçalho
        if (dataFile.size() == 0) {
            dataFile.println("Iteracao,Particula,Distancia,PWM,Erro_Atual,Gbest_Erro");
        }

        dataFile.print(estado.iteracao_atual);
        dataFile.print(","); 
        dataFile.print(estado.particula_atual);
        dataFile.print(",");
        dataFile.print(distancia);
        dataFile.print(",");
        dataFile.print(pwm);
        dataFile.print(",");
        dataFile.print(erro);
        dataFile.print(",");
        dataFile.println(estado.gbest_erro);

        dataFile.close();
    }
}