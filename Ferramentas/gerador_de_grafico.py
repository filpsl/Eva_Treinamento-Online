import pandas as pd
import matplotlib.pyplot as plt
import sys


PASTA = 'exp3'
# --- CONFIGURAÇÃO ---
ARQUIVO_ENTRADA = f'{PASTA}/CONVERG.txt'  # Nome do seu arquivo de log
NOME_DO_GRAFICO = f'{PASTA}/grafico_convergencia.png'
NOME_CSV_LIMPO = f'{PASTA}/dados_limpos.csv'

def gerar_grafico():
    try:
        # 1. Ler o arquivo como texto bruto para garantir flexibilidade
        with open(ARQUIVO_ENTRADA, 'r') as f:
            conteudo = f.read()
    except FileNotFoundError:
        print(f"ERRO: O arquivo '{ARQUIVO_ENTRADA}' não foi encontrado na pasta.")
        return

    # 2. Processamento Robusto dos Dados
    # Substitui quebras de linha por vírgulas e separa tudo
    tokens = conteudo.replace('\n', ',').split(',')
    
    # Filtra apenas o que é número, ignorando cabeçalhos de texto ou espaços vazios
    valores_numericos = []
    for token in tokens:
        token = token.strip()
        if not token: continue # Pula vazio
        try:
            valores_numericos.append(float(token))
        except ValueError:
            continue # Pula textos (ex: cabeçalhos antigos)

    # 3. Organizar em Colunas
    # Sabemos que são 5 colunas: Iteração, Erro, Kp, Ki, Kd
    NUM_COLUNAS = 5
    
    # Verifica se os dados estão completos
    if len(valores_numericos) % NUM_COLUNAS != 0:
        print(f"AVISO: O número de dados ({len(valores_numericos)}) não é múltiplo de {NUM_COLUNAS}.")
        print("Cortando os últimos dados incompletos para evitar erro...")
        sobra = len(valores_numericos) % NUM_COLUNAS
        valores_numericos = valores_numericos[:-sobra]

    # Remodelar a lista plana em uma lista de linhas
    dados_organizados = [
        valores_numericos[i : i + NUM_COLUNAS] 
        for i in range(0, len(valores_numericos), NUM_COLUNAS)
    ]

    # Criar DataFrame
    colunas = ['Iteracao', 'Gbest_Erro', 'Kp', 'Ki', 'Kd']
    df = pd.DataFrame(dados_organizados, columns=colunas)

    if df.empty:
        print("ERRO: Nenhum dado numérico foi encontrado no arquivo.")
        return

    # 4. Agrupamento (Opcional, mas recomendado)
    # Como o PSO pode salvar várias vezes na mesma iteração, pegamos o MENOR erro de cada iteração
    df_plot = df.groupby('Iteracao')['Gbest_Erro'].min().reset_index()

    # 5. Gerar o Gráfico
    plt.figure(figsize=(10, 6))
    
    # Plotar linha com marcadores
    plt.plot(df_plot['Iteracao'], df_plot['Gbest_Erro'], 
             marker='o', linestyle='-', color='#007acc', linewidth=2, markersize=4, label='Melhor Erro Global')

    # Estilização
    plt.title('Convergência do PSO (Otimização PID)', fontsize=14)
    plt.xlabel('Iteração', fontsize=12)
    plt.ylabel('Erro (Função Custo)', fontsize=12)
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    
    # Melhorar visualização dos eixos
    plt.minorticks_on()
    
    # Anotar o melhor valor final
    melhor_erro_final = df_plot['Gbest_Erro'].min()
    plt.annotate(f'Mínimo: {melhor_erro_final:.4f}', 
                 xy=(df_plot.iloc[-1]['Iteracao'], melhor_erro_final), 
                 xytext=(10, 20), textcoords='offset points',
                 arrowprops=dict(arrowstyle="->", color='black'))

    # 6. Salvar Resultados
    plt.savefig(NOME_DO_GRAFICO, dpi=300)
    df.to_csv(NOME_CSV_LIMPO, index=False)

    print("-" * 30)
    print(f"SUCESSO!")
    print(f"1. Gráfico salvo como: {NOME_DO_GRAFICO}")
    print(f"2. Dados limpos salvos como: {NOME_CSV_LIMPO}")
    print(f"3. Melhor erro alcançado: {melhor_erro_final}")
    print("-" * 30)
    
    # Mostrar prévia das últimas linhas
    print("Últimos dados processados:")
    print(df.tail())

if __name__ == "__main__":
    gerar_grafico()