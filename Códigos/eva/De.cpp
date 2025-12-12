#include "De.h"

De::De() {
    erro_da_rodada_atual = 0.0;
    estado.inicializado = false;
    estado.geracao_atual = 0;
    estado.individuo_atual = 0;
    estado.gbest_erro = 10000000.0;
}

float De::randomFloat(float min, float max) {
    return min + (float)random(0, 10000) / 10000.0 * (max - min);
}

void De::inicializar() {
    Serial.println(F("DE: Inicializando nova populacao..."));
    
    estado.geracao_atual = 0;
    estado.individuo_atual = 0;
    estado.gbest_erro = 10000000.0;

    for (int i = 0; i < NUM_PARTICULAS; i++) {
        // Inicializa população aleatória
        estado.populacao[i][0] = randomFloat(KP_MIN, KP_MAX); // Kp
        estado.populacao[i][1] = randomFloat(KI_MIN, KI_MAX); // Ki
        estado.populacao[i][2] = randomFloat(KD_MIN, KD_MAX); // Kd
        
        estado.custos[i] = 10000000.0; // Custo infinito antes de testar
    }
    
    estado.inicializado = true;
    salvarEstado();
}

void De::limitarParametros(float* vetor) {
    if (vetor[0] < KP_MIN) vetor[0] = KP_MIN;
    if (vetor[0] > KP_MAX) vetor[0] = KP_MAX;
    
    if (vetor[1] < KI_MIN) vetor[1] = KI_MIN;
    if (vetor[1] > KI_MAX) vetor[1] = KI_MAX;
    
    if (vetor[2] < KD_MIN) vetor[2] = KD_MIN;
    if (vetor[2] > KD_MAX) vetor[2] = KD_MAX;
}

// O Coração do DE: Mutação + Crossover
void De::gerarVetorTeste(int i) {
    int r1, r2, r3;
    
    // 1. Seleciona 3 agentes aleatórios distintos e diferentes do atual (i)
    do { r1 = random(0, NUM_PARTICULAS); } while (r1 == i);
    do { r2 = random(0, NUM_PARTICULAS); } while (r2 == i || r2 == r1);
    do { r3 = random(0, NUM_PARTICULAS); } while (r3 == i || r3 == r1 || r3 == r2);

    // Índice aleatório para garantir que pelo menos 1 parâmetro mude (Crossover)
    int j_rand = random(0, NUM_DIMENSOES);

    for (int j = 0; j < NUM_DIMENSOES; j++) {
        // Crossover Binomial
        if (randomFloat(0.0, 1.0) < CR_CROSS || j == j_rand) {
            // Mutação: V = X_r1 + F * (X_r2 - X_r3)
            estado.vetor_teste[j] = estado.populacao[r1][j] + F_WEIGHT * (estado.populacao[r2][j] - estado.populacao[r3][j]);
        } else {
            // Mantém valor original
            estado.vetor_teste[j] = estado.populacao[i][j];
        }
    }
    
    limitarParametros(estado.vetor_teste);
}

void De::getParametrosAtuais(float &kp, float &ki, float &kd) {
    int i = estado.individuo_atual;

    if (estado.geracao_atual == 0) {
        // Na primeira geração, apenas testamos a população inicial criada aleatoriamente
        kp = estado.populacao[i][0];
        ki = estado.populacao[i][1];
        kd = estado.populacao[i][2];
    } else {
        // Nas gerações seguintes, precisamos gerar um DESAFIANTE (Trial Vector)
        // para competir contra o indivíduo atual.
        // Como o getParametros pode ser chamado várias vezes, idealmente já teríamos gerado.
        // Mas vamos gerar aqui caso seja a primeira chamada do ciclo.
        
        // OBS: Geramos o vetor teste logicamente antes de enviar para o robô.
        // Para simplificar, recalculamos sempre que pede, pois é determinístico com semente fixa
        // mas como usamos random(), vamos confiar que o fluxo proximaParticula garante a ordem.
        
        // Nota: A geração real acontece aqui para ser enviada ao robô
        gerarVetorTeste(i); 
        
        kp = estado.vetor_teste[0];
        ki = estado.vetor_teste[1];
        kd = estado.vetor_teste[2];
    }
}

void De::setErroDaRodada(float erro) {
    erro_da_rodada_atual = erro;
    int i = estado.individuo_atual;

    // Geração 0: Apenas preenchemos os custos iniciais
    if (estado.geracao_atual == 0) {
        estado.custos[i] = erro;
        
        // Verifica se é o melhor global
        if (erro < estado.gbest_erro) {
            estado.gbest_erro = erro;
            estado.gbest_pos[0] = estado.populacao[i][0];
            estado.gbest_pos[1] = estado.populacao[i][1];
            estado.gbest_pos[2] = estado.populacao[i][2];
            Serial.print(F("DE: Novo Gbest (Gen 0)! Erro: "));
            Serial.println(estado.gbest_erro);
        }
    } 
    else {
        // Geração > 0: SELEÇÃO
        // O robô acabou de rodar com o vetor_teste. O erro recebido é dele.
        // Comparamos com o custo do "pai" (estado.custos[i]).
        
        Serial.print(F("DE: Comparando Teste (")); Serial.print(erro);
        Serial.print(F(") vs Alvo (")); Serial.print(estado.custos[i]); Serial.println(F(")"));

        if (erro < estado.custos[i]) {
            // O filho é melhor! Substitui o pai na população.
            Serial.println(F("DE: Evolucao! Filho substituiu pai."));
            estado.custos[i] = erro;
            estado.populacao[i][0] = estado.vetor_teste[0];
            estado.populacao[i][1] = estado.vetor_teste[1];
            estado.populacao[i][2] = estado.vetor_teste[2];

            // Verifica Gbest
            if (erro < estado.gbest_erro) {
                estado.gbest_erro = erro;
                estado.gbest_pos[0] = estado.vetor_teste[0];
                estado.gbest_pos[1] = estado.vetor_teste[1];
                estado.gbest_pos[2] = estado.vetor_teste[2];
                Serial.println(F("DE: Novo Gbest Encontrado!"));
            }
        } else {
            Serial.println(F("DE: Pai manteve a posicao."));
        }
    }
}

void De::proximaParticula() {
    estado.individuo_atual++;

    if (estado.individuo_atual >= NUM_PARTICULAS) {
        estado.individuo_atual = 0;
        estado.geracao_atual++;
        Serial.print(F("DE: Fim da geracao "));
        Serial.println(estado.geracao_atual);
    }
}

bool De::isConcluido() {
    return (estado.geracao_atual >= MAX_ITERACOES);
}

// --- PERSISTÊNCIA ---
void De::salvarEstado() {
    if (SD.exists(DE_DADOS_BIN)) SD.remove(DE_DADOS_BIN);
    
    File arquivo = SD.open(DE_DADOS_BIN, FILE_WRITE);
    if (arquivo) {
        arquivo.write((uint8_t *)&estado, sizeof(estado));
        arquivo.close();
    } else {
        Serial.println(F("DE ERRO: Falha salvar BIN!"));
    }
}

bool De::carregarEstado() {
    if (!SD.exists(DE_DADOS_BIN)) return false;

    File arquivo = SD.open(DE_DADOS_BIN, FILE_READ);
    if (arquivo) {
        arquivo.read((uint8_t *)&estado, sizeof(estado));
        arquivo.close();
        if (estado.inicializado) {
            Serial.println(F("DE: Save carregado."));
            imprimirStatus();
            return true;
        }
    }
    return false;
}

void De::imprimirStatus() {
    Serial.print(F("--- STATUS DE ---\n"));
    Serial.print(F("Geracao: ")); Serial.println(estado.geracao_atual);
    Serial.print(F("Gbest Erro: ")); Serial.println(estado.gbest_erro);
    Serial.print(F("Gbest [Kp,Ki,Kd]: ["));
    Serial.print(estado.gbest_pos[0]); Serial.print(F(", "));
    Serial.print(estado.gbest_pos[1]); Serial.print(F(", "));
    Serial.print(estado.gbest_pos[2]); Serial.println(F("]"));
}

void De::salvarLog(float distancia, float pwm, float erro) {
    File dataFile = SD.open(DE_DADOS, FILE_WRITE);
    if (dataFile) {
        if (dataFile.size() == 0) dataFile.println("Ger,Ind,Dist,PWM,Erro,Gbest_Erro");
        
        dataFile.print(estado.geracao_atual); dataFile.print(",");
        dataFile.print(estado.individuo_atual); dataFile.print(",");
        dataFile.print(distancia); dataFile.print(",");
        dataFile.print(pwm); dataFile.print(",");
        dataFile.print(erro); dataFile.print(",");
        dataFile.println(estado.gbest_erro);
        dataFile.close();
    }
}

void De::salvarConvergencia() {
    File dataFile = SD.open(DE_CONVERGENCIA, FILE_WRITE);
    if (dataFile) {
        if (dataFile.size() == 0) dataFile.println("Geracao,Gbest_Erro,Kp,Ki,Kd");
        
        dataFile.print(estado.geracao_atual); dataFile.print(",");
        dataFile.print(estado.gbest_erro); dataFile.print(",");
        dataFile.print(estado.gbest_pos[0]); dataFile.print(",");
        dataFile.print(estado.gbest_pos[1]); dataFile.print(",");
        dataFile.println(estado.gbest_pos[2]);
        dataFile.close();
    }
}

void De::apagarDados() {
    if(SD.exists(DE_DADOS_BIN)) SD.remove(DE_DADOS_BIN);
    if(SD.exists(DE_DADOS)) SD.remove(DE_DADOS);
    if(SD.exists(DE_CONVERGENCIA)) SD.remove(DE_CONVERGENCIA);
    Serial.println(F("DE: Dados apagados."));
}