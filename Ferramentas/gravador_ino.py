import serial
import time
import sys
import os

# --- CONFIGURAÇÕES ---
PORTA_SERIAL = '/dev/ttyUSB0'  # <--- CONFIRA SUA PORTA (No Linux: /dev/ttyUSB0)
BAUD_RATE = 115200

# Arquivo que você quer enviar para o robô (pode mudar aqui)
PASTA = "exp1"
# ARQUIVO_NO_PC = f"{PASTA}/pso_data.bin"
ARQUIVO_NO_PC = f"{PASTA}/DADOS.txt"
# ARQUIVO_NO_PC = f"{PASTA}/CONVERG.txt" 

def conectar():
    try:
        print(f"Conectando ao Arduino na porta {PORTA_SERIAL}...")
        ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=2)
        time.sleep(3) # DTR Reset (Essencial para Arduino Nano/Uno)
        # ser.reset_input_buffer()
        return ser
    except Exception as e:
        print(f"Erro de conexão: {e}")
        sys.exit(1)

def esperar_resposta(ser, esperada, timeout=5):
    inicio = time.time()
    while True:
        if time.time() - inicio > timeout:
            return False, "Timeout"
        
        linha = ser.readline().decode().strip()
        if linha:
            # print(f"DEBUG: {linha}") # Descomente se der erro
            if esperada in linha:
                return True, linha
            if "ERRO" in linha:
                return False, linha

def upload(ser, caminho_no_pc):
    if not os.path.exists(caminho_no_pc):
        print(f"ERRO: O arquivo '{caminho_no_pc}' não existe no PC.")
        return

    # --- A CORREÇÃO MÁGICA ---
    # Pega apenas o nome do arquivo (ex: "pso_data.bin") tirando a pasta ("exp1/")
    # Isso garante que grave na raiz do SD, onde o robô consegue ler.
    nome_arquivo_sd = os.path.basename(caminho_no_pc)
    
    tamanho = os.path.getsize(caminho_no_pc)
    print(f"--> Preparando para gravar '{nome_arquivo_sd}' ({tamanho} bytes)...")
    print(f"    (Lendo de: {caminho_no_pc})")

    # 1. Envia comando de gravação com o NOME LIMPO
    ser.write(f"GRAVAR {nome_arquivo_sd}\n".encode())

    # 2. Espera Arduino estar pronto para receber o tamanho
    sucesso, msg = esperar_resposta(ser, "AGUARDANDO_TAMANHO")
    if not sucesso:
        print(f"Falha no Handshake 1: {msg}")
        return

    # 3. Envia o tamanho
    ser.write(f"{tamanho}\n".encode())

    # 4. Espera confirmação para enviar dados
    sucesso, msg = esperar_resposta(ser, "OK_MANDE_DADOS")
    if not sucesso:
        print(f"Falha no Handshake 2: {msg}")
        return

    print("--> Enviando dados...")
    
    # 5. Envio do conteúdo (MODO SEGURO)
    with open(caminho_no_pc, 'rb') as f:
        enviados = 0
        while True:
            # REDUZIDO: 32 bytes (Metade do buffer do Arduino) para evitar transbordo
            chunk = f.read(32) 
            if not chunk:
                break
            
            ser.write(chunk)
            enviados += len(chunk)
            
            # AUMENTADO: 0.010s (10ms) para garantir que o SD gravou
            time.sleep(0.010) 
            
            percent = (enviados / tamanho) * 100
            print(f"    Progresso: {percent:.1f}%", end='\r')
            
    # 6. Espera confirmação final
    sucesso, msg = esperar_resposta(ser, "SUCESSO_GRAVACAO_CONCLUIDA", timeout=10)
    
    if sucesso:
        print("\n[V] SUCESSO! Arquivo restaurado na raiz do cartão.")
    else:
        print(f"\n[X] Falha na verificação final: {msg}")

if __name__ == "__main__":
    arduino = conectar()
    
    # Espera o Arduino inicializar e dizer que está pronto
    print("Aguardando Arduino inicializar...")
    sucesso, msg = esperar_resposta(arduino, "GRAVADOR_PRONTO")
    
    if sucesso:
        print("Arduino Pronto.")
        upload(arduino, ARQUIVO_NO_PC)
    else:
        print("O Arduino não respondeu 'GRAVADOR_PRONTO'. Verifique o código carregado.")
    
    arduino.close()