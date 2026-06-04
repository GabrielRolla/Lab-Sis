import serial
import time

# =======================================================
# ATENÇÃO: Troque 'COM3' pela porta que o seu ESP32 usa 
# (Ex: 'COM5' no Windows ou '/dev/ttyUSB0' no Linux/Mac)
# =======================================================
COM_PORT = 'COM3' 
BAUD_RATE = 115200

print(f"Conectando na porta {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Erro ao abrir a porta: {e}")
    print("DICA: Feche o 'Monitor Serial' da Arduino IDE! A porta só aceita uma conexão por vez.")
    exit()

print("Aguardando o ESP32 enviar o arquivo (Reinicie o ESP32 agora no botão da placa!)...")

# 1. Fica ouvindo a porta até o ESP32 mandar o marcador de início
while True:
    line = ser.readline()
    if b"---START_OF_WAV---" in line:
        break

print("Recebendo dados brutos de áudio... Isso vai demorar alguns segundos.")

# 2. Lê os bytes binários até encontrar o marcador de fim
audio_data = bytearray()
while True:
    # Lê tudo o que estiver na fila da porta USB
    chunk = ser.read(ser.in_waiting or 1)
    audio_data.extend(chunk)
    
    # Se achou a assinatura de finalização, interrompe o loop
    if b"---END_OF_WAV---" in audio_data:
        break

# 3. Limpa o marcador de fim dos dados do áudio
audio_data = audio_data.split(b"---END_OF_WAV---")[0]

ser.close()

# 4. Salva o arquivo no seu computador
with open("audio_resgate.wav", "wb") as f:
    f.write(audio_data)

print(f"Sucesso! Arquivo 'audio_resgate.wav' salvo na sua pasta com {len(audio_data)} bytes.")