import serial
import time
import sys
import os

# --- CONFIGURAÇÕES ---
PORTA_SERIAL = '/dev/ttyUSB0'  # <--- MUDE ISSO PARA SUA PORTA (No Linux: /dev/ttyUSB0)
BAUD_RATE = 115200
PASTA = "exp5_de" # Onde vai salvar no PC

# Lista de arquivos que você quer baixar do Arduino
# ARQUIVOS_PARA_BAIXAR = [
#     "DADOS.txt",
#     "pso_data.bin",
#     "CONVERG.txt" # Adicione outros se precisar
# ]

ARQUIVOS_PARA_BAIXAR = [
    "DE_DADOS.txt",
    "de_data.bin",
    "DE_CONV.txt" # Adicione outros se precisar
]

def conectar_arduino():
    try:
        print(f"Tentando conectar em {PORTA_SERIAL}...")
        ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=2)
        time.sleep(3) # Espera o Arduino reiniciar (DTR reset)
        
        # Limpa qualquer lixo que o Arduino tenha mandado na inicialização
        ser.reset_input_buffer()
        print("Conectado! Iniciando downloads...\n")
        return ser
    except serial.SerialException:
        print(f"ERRO: Não foi possível abrir a porta {PORTA_SERIAL}.")
        print("DICA: Feche o Monitor Serial do Arduino IDE antes de rodar este script.")
        sys.exit(1)

def baixar_arquivo(ser, nome_arquivo):
    print(f"--> Pedindo '{nome_arquivo}'...")
    
    # Envia o comando para o Arduino (Nome + Quebra de linha)
    ser.write(f"{nome_arquivo}\n".encode())
    
    # Aguarda resposta do cabeçalho (Ex: "OK 1024" ou "ERRO_404")
    resposta = ser.readline().decode().strip()
    
    if "ERRO" in resposta:
        print(f"    [X] Falha: O Arduino respondeu {resposta} (Arquivo não existe?)")
        return
    
    if not resposta.startswith("OK"):
        print(f"    [X] Erro de Protocolo: Resposta estranha '{resposta}'")
        return

    # Extrai o tamanho do arquivo
    try:
        tamanho_total = int(resposta.split(" ")[1])
    except:
        print("    [X] Erro ao ler tamanho do arquivo.")
        return

    print(f"    [i] Tamanho detectado: {tamanho_total} bytes. Baixando...")

    # Cria a pasta se não existir
    if not os.path.exists(PASTA):
        os.makedirs(PASTA)

    caminho_completo = os.path.join(PASTA, nome_arquivo)

    # Loop de leitura binária
    bytes_recebidos = 0
    with open(caminho_completo, 'wb') as f:
        while bytes_recebidos < tamanho_total:
            # Lê em blocos ou o que faltar
            falta = tamanho_total - bytes_recebidos
            # Lê do buffer serial
            chunk = ser.read(min(falta, 2048)) 
            
            if not chunk:
                print("    [X] Timeout: Conexão perdeu dados no meio do caminho.")
                break
            
            f.write(chunk)
            bytes_recebidos += len(chunk)
            
            # Barra de progresso simples
            percent = (bytes_recebidos / tamanho_total) * 100
            print(f"    Progresso: {percent:.1f}%", end='\r')

    print(f"\n    [V] Sucesso! Salvo em: {caminho_completo}\n")

# --- EXECUÇÃO ---
if __name__ == "__main__":
    arduino = conectar_arduino()
    
    for arquivo in ARQUIVOS_PARA_BAIXAR:
        baixar_arquivo(arduino, arquivo)
        time.sleep(1) # Respira entre arquivos
        arduino.reset_input_buffer()

    arduino.close()
    print("Todos os downloads finalizados.")