#ifndef FUNCAO_CUSTO_H
#define FUNCAO_CUSTO_H

#include <Arduino.h>

class FuncaoCusto {
public:
    virtual ~FuncaoCusto() {}

    // Zera os acumuladores antes de começar uma nova partícula
    virtual void reset() = 0;

    // Chamado a cada ciclo do loop() durante a execução
    // erro: Setpoint - ValorAtual
    // tempo_decorrido_ms: Tempo em ms desde que o robô arrancou (millis() - t_inicio)
    virtual void acumular(float erro, unsigned long tempo_decorrido_ms) = 0;

    // Retorna o valor final para o Otimizador
    virtual float getCustoFinal() = 0;

    // Apenas para log no SD Card (saber qual função foi usada)
    virtual const char* getNome() = 0;
};

#endif