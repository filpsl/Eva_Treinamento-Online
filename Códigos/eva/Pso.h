#ifndef PSO_H
#define PSO_H

#include "Otimizador.h"
#include <SD.h>
#include <Arduino.h>

// Nomes dos arquivos do SD
#define DADOS_BIN     "pso_data.bin"
#define CONVERGENCIA  "CONVERG.txt"
#define DADOS         "DADOS.txt"

// --- CONFIGURAÇÕES DO PSO ---
#define NUM_PARTICULAS 7
#define NUM_DIMENSOES 3   // 3 para PID (Kp, Ki, Kd)
#define MAX_ITERACOES 50

// Limites dos parâmetros PID
#define KP_MIN 1.0
#define KP_MAX 5.0
#define KI_MIN 0.0
#define KI_MAX 2.0
#define KD_MIN 0.0
#define KD_MAX 1.0

// Constantes do PSO
#define C1 1.5    // Cognitivo
#define C2 1.5    // Social

class Pso : public Otimizador {
private:
    // Estrutura de Checkpoint (Binário)
    struct PsoState {
        int iteracao_atual;
        int particula_atual;
        float x[NUM_PARTICULAS][NUM_DIMENSOES];      // Posição
        float v[NUM_PARTICULAS][NUM_DIMENSOES];      // Velocidade
        float pbest_pos[NUM_PARTICULAS][NUM_DIMENSOES];
        float pbest_erro[NUM_PARTICULAS];
        float gbest_pos[NUM_DIMENSOES];
        float gbest_erro;
        bool inicializado;
        float W = 0.9;    // Inércia
        float W_f = 0.3;   // Inércia Final
        float W_passo = (W_f - W) / MAX_ITERACOES; // Passo da inércia
    };

    PsoState estado;

    float erro_da_rodada_atual;

    // Métodos privados
    float randomFloat(float min, float max);
    void limitarPosicao(int p_idx);

public:
    
    Pso(); // Construtor
    
    // --- Implementação da Interface Otimizador ---
    void inicializar() override;
    void getParametrosAtuais(float &kp, float &ki, float &kd) override;
    void setErroDaRodada(float erro) override;
    void proximaParticula() override;
    bool isConcluido() override;
    
    // Persistência (Checkpoint)
    void salvarEstado() override;
    bool carregarEstado() override;
    
    // Log Legível (Excel/CSV) - A PEÇA QUE FALTAVA
    void salvarLog(float dist, float pwm, float erro) override;
    void salvarConvergencia() override;

    void apagarDados() override;
    
    // Debug
    void imprimirStatus() override;
};

#endif