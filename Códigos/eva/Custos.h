#ifndef CUSTOS_H
#define CUSTOS_H

#include "FuncaoCusto.h"
#include <math.h>

// --- 1. MSE: Erro Quadrático Médio ---
// Fórmula: (1/N) * Σ(erro²)
class CustoMSE : public FuncaoCusto {
private:
    float soma_quadrados;
    unsigned long contagem;

public:
    void reset() override {
        soma_quadrados = 0.0;
        contagem = 0;
    }

    void acumular(float erro, unsigned long tempo_decorrido_ms) override {
        soma_quadrados += (erro * erro);
        contagem++;
    }

    float getCustoFinal() override {
        if (contagem == 0) return 1000000.0; // Evita divisão por zero
        return soma_quadrados / (float)contagem;
    }
    
    const char* getNome() override { return "MSE"; }
};

// --- 2. IAE: Integral do Erro Absoluto ---
// Fórmula: Σ|erro|
class CustoIAE : public FuncaoCusto {
private:
    float soma_absoluta;

public:
    void reset() override {
        soma_absoluta = 0.0;
    }

    void acumular(float erro, unsigned long tempo_decorrido_ms) override {
        soma_absoluta += fabs(erro);
    }

    float getCustoFinal() override {
        return soma_absoluta;
    }

    const char* getNome() override { return "IAE"; }
};

// --- 3. ITAE: Integral do Erro Absoluto Ponderado pelo Tempo ---
// Fórmula: Σ (tempo * |erro|)
// Penaliza oscilações tardias. Força o robô a estabilizar rápido.
class CustoITAE : public FuncaoCusto {
private:
    float soma_ponderada;

public:
    void reset() override {
        soma_ponderada = 0.0;
    }

    void acumular(float erro, unsigned long tempo_decorrido_ms) override {
        // Converte ms para segundos para o número não explodir
        float t_segundos = tempo_decorrido_ms / 1000.0; 
        soma_ponderada += t_segundos * fabs(erro);
    }

    float getCustoFinal() override {
        return soma_ponderada;
    }

    const char* getNome() override { return "ITAE"; }
};

#endif