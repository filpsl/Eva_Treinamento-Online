#ifndef DE_H
#define DE_H

#include "Otimizador.h"
#include "config.h"
#include <SD.h>
#include <Arduino.h>

// Nomes dos arquivos do SD
#define DE_DADOS_BIN     "de_data.bin"
#define DE_CONVERGENCIA  "DE_CONV.txt"
#define DE_DADOS         "DE_DADOS.txt"

// Parâmetros do Algoritmo DE
#define F_WEIGHT 0.6      // Fator de Mutação (0.5 a 0.9)
#define CR_CROSS 0.8      // Taxa de Crossover (0.0 a 1.0)

class De : public Otimizador {
private:
    // Estrutura de Checkpoint (Binário)
    struct DeState {
        int geracao_atual;
        int individuo_atual;
        
        // População Principal (Target Vectors)
        float populacao[NUM_PARTICULAS][NUM_DIMENSOES];
        float custos[NUM_PARTICULAS]; // Custo de cada indivíduo
        
        // Vetor de Teste Atual (Trial Vector)
        // O DE gera um candidato, testamos ele, e depois decidimos se ele entra na população
        float vetor_teste[NUM_DIMENSOES];
        
        // Melhor Global (Apenas para histórico/log)
        float gbest_pos[NUM_DIMENSOES];
        float gbest_erro;
        
        bool inicializado;
    };

    DeState estado;
    float erro_da_rodada_atual;

    // Métodos privados auxiliares
    float randomFloat(float min, float max);
    void limitarParametros(float* vetor);
    void gerarVetorTeste(int indice_alvo);

public:
    De(); // Construtor

    // --- Implementação da Interface Otimizador ---
    void inicializar() override;
    void getParametrosAtuais(float &kp, float &ki, float &kd) override;
    void setErroDaRodada(float erro) override;
    void  proximaParticula() override;
    bool isConcluido() override;

    // Persistência
    void salvarEstado() override;
    bool carregarEstado() override;

    // Logs
    void salvarLog(float dist, float pwm, float erro) override;
    void salvarConvergencia() override;
    void apagarDados() override;
    void imprimirStatus() override;
};

#endif