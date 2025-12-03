#ifndef OTIMIZADOR_H
#define OTIMIZADOR_H

#include <Arduino.h>

class Otimizador {
public:
    virtual ~Otimizador() {}

    // --- MÉTODOS DE CONTROLE ---
    virtual void inicializar() = 0;
    virtual void getParametrosAtuais(float &kp, float &ki, float &kd) = 0;
    virtual void setErroDaRodada(float erro) = 0;
    virtual void proximaParticula() = 0;
    virtual bool isConcluido() = 0;

    // --- PERSISTÊNCIA E LOGS ---
    
    // 1. Checkpoint Binário (Salva o Cérebro para não perder se acabar bateria)
    // NÃO recebe parâmetros, pois salva a struct interna inteira.
    virtual void salvarEstado() = 0;
    virtual bool carregarEstado() = 0;

    // 2. Log CSV/Excel (Salva o Relatório para você ler depois)
    // NOVO: Recebe os dados da rodada para gravar no DADOS.txt
    virtual void salvarLog(float dist, float pwm, float erro) = 0;
    
    virtual void imprimirStatus() = 0;
};

#endif