/*
 * ============================================================
 * Sensor Vibroacustico - INMP441 + Piezo + OLED + SD + Botoes
 * Projeto: Localizador Acustico de Vitimas
 *
 * Revisao:
 * - Sem liga/desliga logico.
 * - O ESP32 inicia ativo automaticamente.
 * - GPIO 13 -> troca modo acustico/vibracional.
 * - GPIO 4  -> inicia/pausa/retoma gravacao.
 * - GPIO 27 -> sem funcao no software.
 * - GPIO 35 -> potenciometro 10k para sensibilidade.
 * - Piezo usa ADC1 legacy para evitar conflito ADC driver_ng x I2S legacy.
 * - Serial Monitor imprime apenas mudancas de estado e eventos importantes.
 * - DETECTOR VAD IMPLEMENTADO: Envelope de Energia + ZCR + Fator de Tempo (Anti-Palmas).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <driver/adc.h>

#include "interface_estado.h"
#include "interface_display.h"

// ============================================================
// OLED I2C
// ============================================================
#define OLED_SDA 21
#define OLED_SCL 22

// ============================================================
// Botoes
// ============================================================
#define BOTAO_GRAVACAO   4
#define BOTAO_MODO       13
#define BOTAO_SEM_FUNCAO 27

const unsigned long DEBOUNCE_MS = 220;

// ============================================================
// LED
// ============================================================
#define LED_PIN 2

// ============================================================
// INMP441 - I2S
// ============================================================
#define I2S_WS    25
#define I2S_SCK   14
#define I2S_SD    26
#define I2S_PORT  I2S_NUM_0

#define BUFFER_LEN 128

int32_t samplesBuffer[BUFFER_LEN];
int16_t writeBuffer[BUFFER_LEN];

bool i2sAtivo = false;

// ============================================================
// Piezo - ADC1 legacy
// ============================================================
#define PIEZO_ADC_CHANNEL ADC1_CHANNEL_6 // GPIO34

// ============================================================
// Potenciometro de sensibilidade - ADC1 legacy
// ============================================================
#define POT_SENSIBILIDADE_ADC_CHANNEL ADC1_CHANNEL_7 // GPIO35

// ============================================================
// Cartao SD - SPI
// ============================================================
#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  23
#define SPI_MOSI  19

const char* AUDIO_FILE_PATH = "/gravacao.wav";

const int SAMPLE_RATE = 16000;
const int WAV_HEADER_SIZE = 44;
const int CHANNELS = 1;
const int BITS_PER_SAMPLE = 16;
const int BYTES_PER_SAMPLE = BITS_PER_SAMPLE / 8;

bool sdIniciado = false;
File audioFile;

bool gravacaoIniciada = false;
bool gravacaoPausada = false;
uint32_t totalAudioBytes = 0;

// ============================================================
// Estado do sistema
// ============================================================
enum ModoSensor {
  MODO_ACUSTICO_INMP441,
  MODO_VIBRACIONAL_PIEZO
};

ModoSensor modoSensorAtual = MODO_ACUSTICO_INMP441;

EstadoSistema estadoAtual = ESTADO_RECEBENDO_SINAIS;
ModoOperacao modoAtual = MODO_ACUSTICO;

uint8_t nivelSinalAtual = 0;

unsigned long ultimoRefreshDisplay = 0;
const unsigned long INTERVALO_DISPLAY_MS = 100;

// ============================================================
// Configuracoes Piezo
// ============================================================
const int PIEZO_THRESHOLD_MIN = 40;   // Mais sensivel
const int PIEZO_THRESHOLD_MAX = 600;  // Menos sensivel
const int PIEZO_PARA_100 = 900;

float envelopePiezo = 0.0f;
const float ALPHA_PIEZO = 0.20f;

bool impactoDetectado = false;
bool ultimoImpactoDetectado = false;

// ============================================================
// Filtro INMP441 e Detector de Voz (VAD)
// ============================================================
const float ALPHA_300 = 0.11f;
const float ALPHA_3000 = 0.54f;

float grave1_lastY = 0.0f;
float grave2_lastY = 0.0f;
float voz_lp1_lastY = 0.0f;
float voz_lp2_lastY = 0.0f;

float envelopeVoz = 0.0f;
const float ALPHA_ENVELOPE_ATAQUE = 0.18f;
const float ALPHA_ENVELOPE_RELEASE = 0.04f;

float ruidoFundo = 0.0f;
const float ALPHA_RUIDO = 0.01f;

const float MARGEM_DETECCAO_MIN = 600.0f;   // Mais sensivel
const float MARGEM_DETECCAO_MAX = 4000.0f;  // Menos sensivel
const float LIMIAR_MINIMO_VOZ_MIN = 600.0f;
const float LIMIAR_MINIMO_VOZ_MAX = 4000.0f;

// --- Parametros do Zero-Crossing Rate (ZCR) ---
const int ZCR_MIN = 4;   
const int ZCR_MAX = 50;  
float last_filtered_sample = 0.0f; 

const float ENVELOPE_PARA_100 = 10000.0f;
const float GANHO_INMP441 = 1.0f;

bool vozDetectada = false;
bool ultimaVozDetectada = false;

// ============================================================
// Leitura do potenciometro de sensibilidade
// ============================================================
float sensibilidadeFiltrada = 0.5f;
const float ALPHA_POTENCIOMETRO = 0.12f;

// ============================================================
// Controle dos botoes por borda de descida
// ============================================================
bool estadoAnteriorBotaoModo = HIGH;
bool estadoAnteriorBotaoGravacao = HIGH;

unsigned long ultimoDebounceModo = 0;
unsigned long ultimoDebounceGravacao = 0;

// ============================================================
// Prototipos
// ============================================================
void processarBotoes();

void trocarModo();
void entrarModoAcustico();
void entrarModoVibracional();

void setupI2S();
void desligarI2S();
void resetarEstadosAudio();

uint8_t lerNivelDoSinal();
uint8_t lerNivelVozINMP441();
uint8_t lerNivelPiezo();

float lerSensibilidadePotenciometro();
float calcularLimiarPorSensibilidade(float sensibilidade, float menorLimiar, float maiorLimiar);
int calcularPiezoThreshold(float sensibilidade);
float calcularMargemDeteccao(float sensibilidade);
float calcularLimiarMinimoVoz(float sensibilidade);

float aplicarFiltroVoz(float x);
float atualizarEnvelopeVoz(float volumeAtual);
void atualizarRuidoFundo(float volumeAtual);

uint8_t mapearParaNivel(float valor, float valorPara100);

bool iniciarSD();
void alternarGravacao();
void iniciarGravacao();
void finalizarGravacao();
void writeWavHeader(File& file, uint32_t dataSize);

int16_t limitarInt16(float valor);

void imprimirEstadoInicial();
void imprimirMudancaVoz();
void imprimirMudancaImpacto();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BOTAO_GRAVACAO, INPUT_PULLUP);
  pinMode(BOTAO_MODO, INPUT_PULLUP);
  pinMode(BOTAO_SEM_FUNCAO, INPUT_PULLUP);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(PIEZO_ADC_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(POT_SENSIBILIDADE_ADC_CHANNEL, ADC_ATTEN_DB_11);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!displayIniciar()) {
    Serial.println("[ERRO] Falha ao iniciar display SSD1306");
    while (true) {
      delay(1000);
    }
  }

  imprimirEstadoInicial();
  entrarModoAcustico();
  displayRenderizar(estadoAtual, modoAtual, 0, (uint8_t)(lerSensibilidadePotenciometro() * 100.0f));
}

// ============================================================
// Loop
// ============================================================
void loop() {
  processarBotoes();

  nivelSinalAtual = lerNivelDoSinal();

  unsigned long agora = millis();

  if (agora - ultimoRefreshDisplay >= INTERVALO_DISPLAY_MS) {
    ultimoRefreshDisplay = agora;
    uint8_t sensibilidadePers = (uint8_t)(lerSensibilidadePotenciometro() * 100.0f);
    displayRenderizar(estadoAtual, modoAtual, nivelSinalAtual, vozDetectada, gravacaoIniciada ? (gravacaoPausada ? GRAVACAO_PAUSADA : GRAVACAO_GRAVANDO) : GRAVACAO_PARADA, sensibilidadePers);
  }

  delay(10);
}

// ============================================================
// Prints principais
// ============================================================
void imprimirEstadoInicial() {
  Serial.println();
  Serial.println("=====================================");
  Serial.println(" SISTEMA INICIADO");
  Serial.println("=====================================");
  Serial.println("GPIO 13 -> trocar modo");
  Serial.println("GPIO 4  -> iniciar/pausar/retomar gravacao");
  Serial.println("GPIO 27 -> sem funcao no software");
  Serial.println("GPIO 35 -> potenciometro de sensibilidade");
  Serial.println("Modo inicial: ACUSTICO / INMP441");
  Serial.println("Serial: apenas mudancas de estado");
  Serial.println("=====================================");
  Serial.println();
}

void imprimirMudancaVoz() {
  if (vozDetectada == ultimaVozDetectada) {
    return;
  }
  ultimaVozDetectada = vozDetectada;

  if (vozDetectada) {
    Serial.println("[VOZ] Voz detectada");
  } else {
    Serial.println("[VOZ] Voz nao detectada");
  }
}

void imprimirMudancaImpacto() {
  if (impactoDetectado == ultimoImpactoDetectado) {
    return;
  }
  ultimoImpactoDetectado = impactoDetectado;

  if (impactoDetectado) {
    Serial.println("[PIEZO] Impacto detectado");
  } else {
    Serial.println("[PIEZO] Impacto encerrado");
  }
}

// ============================================================
// Botoes
// ============================================================
void processarBotoes() {
  unsigned long agora = millis();
  bool leituraModo = digitalRead(BOTAO_MODO);
  bool leituraGravacao = digitalRead(BOTAO_GRAVACAO);

  if (estadoAnteriorBotaoModo == HIGH && leituraModo == LOW && agora - ultimoDebounceModo > DEBOUNCE_MS) {
    ultimoDebounceModo = agora;
    trocarModo();
  }

  if (estadoAnteriorBotaoGravacao == HIGH && leituraGravacao == LOW && agora - ultimoDebounceGravacao > DEBOUNCE_MS) {
    ultimoDebounceGravacao = agora;
    alternarGravacao();
  }

  estadoAnteriorBotaoModo = leituraModo;
  estadoAnteriorBotaoGravacao = leituraGravacao;
}

// ============================================================
// Troca de modo
// ============================================================
void trocarModo() {
  if (modoSensorAtual == MODO_ACUSTICO_INMP441) {
    entrarModoVibracional();
  } else {
    entrarModoAcustico();
  }
}

void entrarModoAcustico() {
  finalizarGravacao();
  desligarI2S();

  modoSensorAtual = MODO_ACUSTICO_INMP441;
  modoAtual = MODO_ACUSTICO;

  resetarEstadosAudio();
  setupI2S();

  digitalWrite(LED_PIN, LOW);
  Serial.println("[MODO] Acustico / INMP441 ativo");
}

void entrarModoVibracional() {
  finalizarGravacao();
  desligarI2S();

  modoSensorAtual = MODO_VIBRACIONAL_PIEZO;
  modoAtual = MODO_VIBRACIONAL;

  envelopePiezo = 0.0f;
  impactoDetectado = false;
  ultimoImpactoDetectado = false;

  digitalWrite(LED_PIN, LOW);
  Serial.println("[MODO] Vibracional / Piezo ativo");
}

// ============================================================
// Leitura principal
// ============================================================
uint8_t lerNivelDoSinal() {
  if (modoSensorAtual == MODO_ACUSTICO_INMP441) {
    return lerNivelVozINMP441();
  }
  return lerNivelPiezo();
}

// ============================================================
// Potenciometro de sensibilidade
// ============================================================
float lerSensibilidadePotenciometro() {
  int leitura = adc1_get_raw(POT_SENSIBILIDADE_ADC_CHANNEL);
  if (leitura < 0) {
    leitura = 0;
  }
  if (leitura > 4095) {
    leitura = 4095;
  }

  float sensibilidadeAtual = (float)leitura / 4095.0f;
  sensibilidadeFiltrada = (ALPHA_POTENCIOMETRO * sensibilidadeAtual) + ((1.0f - ALPHA_POTENCIOMETRO) * sensibilidadeFiltrada);
  return sensibilidadeFiltrada;
}

float calcularLimiarPorSensibilidade(float sensibilidade, float menorLimiar, float maiorLimiar) {
  if (sensibilidade < 0.0f) {
    sensibilidade = 0.0f;
  }
  if (sensibilidade > 1.0f) {
    sensibilidade = 1.0f;
  }

  return maiorLimiar - ((maiorLimiar - menorLimiar) * sensibilidade);
}

int calcularPiezoThreshold(float sensibilidade) {
  return (int)calcularLimiarPorSensibilidade(sensibilidade, PIEZO_THRESHOLD_MIN, PIEZO_THRESHOLD_MAX);
}

float calcularMargemDeteccao(float sensibilidade) {
  return calcularLimiarPorSensibilidade(sensibilidade, MARGEM_DETECCAO_MIN, MARGEM_DETECCAO_MAX);
}

float calcularLimiarMinimoVoz(float sensibilidade) {
  return calcularLimiarPorSensibilidade(sensibilidade, LIMIAR_MINIMO_VOZ_MIN, LIMIAR_MINIMO_VOZ_MAX);
}

// ============================================================
// I2S
// ============================================================
void setupI2S() {
  if (i2sAtivo) {
    return;
  }

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("[ERRO] i2s_driver_install: ");
    Serial.println(err);
    i2sAtivo = false;
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.print("[ERRO] i2s_set_pin: ");
    Serial.println(err);
    i2s_driver_uninstall(I2S_PORT);
    i2sAtivo = false;
    return;
  }

  err = i2s_zero_dma_buffer(I2S_PORT);
  if (err != ESP_OK) {
    Serial.print("[ERRO] i2s_zero_dma_buffer: ");
    Serial.println(err);
    i2s_driver_uninstall(I2S_PORT);
    i2sAtivo = false;
    return;
  }

  i2sAtivo = true;
}

void desligarI2S() {
  if (!i2sAtivo) {
    return;
  }
  i2s_driver_uninstall(I2S_PORT);
  i2sAtivo = false;
}

void resetarEstadosAudio() {
  grave1_lastY = 0.0f;
  grave2_lastY = 0.0f;
  voz_lp1_lastY = 0.0f;
  voz_lp2_lastY = 0.0f;

  envelopeVoz = 0.0f;
  ruidoFundo = 0.0f;
  
  last_filtered_sample = 0.0f; 

  vozDetectada = false;
  ultimaVozDetectada = false;
}

// ============================================================
// INMP441 + filtro + gravacao + ZCR + Sustentacao
// ============================================================
uint8_t lerNivelVozINMP441() {
  if (!i2sAtivo) {
    digitalWrite(LED_PIN, LOW);
    return 0;
  }

  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, samplesBuffer, sizeof(samplesBuffer), &bytesRead, pdMS_TO_TICKS(300));

  if (result != ESP_OK || bytesRead == 0) {
    vozDetectada = false;
    digitalWrite(LED_PIN, LOW);
    imprimirMudancaVoz();
    return 0;
  }

  int samplesRead = bytesRead / sizeof(int32_t);

  if (samplesRead <= 0) {
    vozDetectada = false;
    digitalWrite(LED_PIN, LOW);
    imprimirMudancaVoz();
    return 0;
  }

  float maxVoz = -999999.0f;
  float minVoz = 999999.0f;
  int zcrCount = 0; 

  for (int i = 0; i < samplesRead; i++) {
    float x = (float)(samplesBuffer[i] >> 16);
    x *= GANHO_INMP441;

    float y = aplicarFiltroVoz(x);

    if ((y >= 0 && last_filtered_sample < 0) || (y < 0 && last_filtered_sample >= 0)) {
        zcrCount++;
    }
    last_filtered_sample = y; 

    if (y > maxVoz) {
      maxVoz = y;
    }
    if (y < minVoz) {
      minVoz = y;
    }

    writeBuffer[i] = limitarInt16(y);
  }

  if (gravacaoIniciada && !gravacaoPausada && audioFile) {
    size_t bytesToWrite = samplesRead * sizeof(int16_t);
    size_t bytesWritten = audioFile.write((const uint8_t*)writeBuffer, bytesToWrite);
    totalAudioBytes += bytesWritten;

    if (bytesWritten != bytesToWrite) {
      Serial.println("[AVISO] Escrita incompleta no SD");
    }
  }

  float volumeVozBruto = maxVoz - minVoz;
  
  atualizarRuidoFundo(volumeVozBruto); 
  atualizarEnvelopeVoz(volumeVozBruto);

  float sensibilidade = lerSensibilidadePotenciometro();
  float margemDeteccao = calcularMargemDeteccao(sensibilidade);
  float limiarMinimoVoz = calcularLimiarMinimoVoz(sensibilidade);

  float limiarAdaptativo = ruidoFundo + margemDeteccao;
  if (limiarAdaptativo < limiarMinimoVoz) {
    limiarAdaptativo = limiarMinimoVoz;
  }

  // --- VAD: Energia + ZCR + Sustentacao ---
  bool energiaSuficiente = envelopeVoz > limiarAdaptativo;
  bool zcrValido = (zcrCount > ZCR_MIN && zcrCount < ZCR_MAX);

  static int contadorVozSustentada = 0; 
  const int BUFFERS_MINIMOS_VOZ = 15; // Ajuste para mais ou menos tempo (15 = ~120ms)

  if (energiaSuficiente && zcrValido) {
      contadorVozSustentada++; 
  } else {
      contadorVozSustentada = 0; 
  }

  if (contadorVozSustentada >= BUFFERS_MINIMOS_VOZ) {
      vozDetectada = true;
      contadorVozSustentada = BUFFERS_MINIMOS_VOZ; 
  } else {
      vozDetectada = false;
  }

  uint8_t nivel = mapearParaNivel(envelopeVoz, ENVELOPE_PARA_100);

  digitalWrite(LED_PIN, vozDetectada ? HIGH : LOW);

  imprimirMudancaVoz();

  return nivel;
}

float aplicarFiltroVoz(float x) {
  float y_grave1 = (ALPHA_300 * x) + ((1.0f - ALPHA_300) * grave1_lastY);
  grave1_lastY = y_grave1;

  float y_grave2 = (ALPHA_300 * y_grave1) + ((1.0f - ALPHA_300) * grave2_lastY);
  grave2_lastY = y_grave2;

  float y_sem_graves = x - y_grave2;

  float y_voz_lp1 = (ALPHA_3000 * y_sem_graves) + ((1.0f - ALPHA_3000) * voz_lp1_lastY);
  voz_lp1_lastY = y_voz_lp1;

  float y_voz_lp2 = (ALPHA_3000 * y_voz_lp1) + ((1.0f - ALPHA_3000) * voz_lp2_lastY);
  voz_lp2_lastY = y_voz_lp2;

  return y_voz_lp2;
}

float atualizarEnvelopeVoz(float volumeAtual) {
  float alpha = volumeAtual > envelopeVoz ? ALPHA_ENVELOPE_ATAQUE : ALPHA_ENVELOPE_RELEASE;
  envelopeVoz = (alpha * volumeAtual) + ((1.0f - alpha) * envelopeVoz);
  return envelopeVoz;
}

void atualizarRuidoFundo(float volumeAtual) {
  if (!vozDetectada) {
    ruidoFundo = (ALPHA_RUIDO * volumeAtual) + ((1.0f - ALPHA_RUIDO) * ruidoFundo);
  }
}

// ============================================================
// Piezo
// ============================================================
uint8_t lerNivelPiezo() {
  int valor = adc1_get_raw(PIEZO_ADC_CHANNEL);
  int piezoThreshold = calcularPiezoThreshold(lerSensibilidadePotenciometro());

  impactoDetectado = valor > piezoThreshold;

  int valorUtil = valor - piezoThreshold;
  if (valorUtil < 0) {
    valorUtil = 0;
  }

  envelopePiezo = (ALPHA_PIEZO * valorUtil) + ((1.0f - ALPHA_PIEZO) * envelopePiezo);

  uint8_t nivel = mapearParaNivel(envelopePiezo, PIEZO_PARA_100);

  digitalWrite(LED_PIN, impactoDetectado ? HIGH : LOW);

  imprimirMudancaImpacto();

  return nivel;
}

// ============================================================
// Gravacao WAV
// ============================================================
bool iniciarSD() {
  if (sdIniciado) {
    return true;
  }

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("[ERRO] Falha ao iniciar cartao SD");
    return false;
  }

  sdIniciado = true;
  Serial.println("[SD] Cartao SD iniciado");

  return true;
}

void alternarGravacao() {
  if (modoSensorAtual != MODO_ACUSTICO_INMP441) {
    Serial.println("[GRAVACAO] Disponivel apenas no modo acustico");
    return;
  }

  if (!gravacaoIniciada) {
    iniciarGravacao();
    return;
  }

  gravacaoPausada = !gravacaoPausada;

  if (gravacaoPausada) {
    Serial.println("[GRAVACAO] Pausada");
  } else {
    Serial.println("[GRAVACAO] Retomada");
  }
}

void iniciarGravacao() {
  if (!iniciarSD()) {
    return;
  }

  if (SD.exists(AUDIO_FILE_PATH)) {
    SD.remove(AUDIO_FILE_PATH);
  }

  audioFile = SD.open(AUDIO_FILE_PATH, FILE_WRITE);

  if (!audioFile) {
    Serial.println("[ERRO] Nao foi possivel criar arquivo WAV");
    return;
  }

  totalAudioBytes = 0;
  gravacaoIniciada = true;
  gravacaoPausada = false;

  writeWavHeader(audioFile, 0);

  Serial.print("[GRAVACAO] Iniciada em ");
  Serial.println(AUDIO_FILE_PATH);
}

void finalizarGravacao() {
  if (!gravacaoIniciada) {
    return;
  }

  Serial.println("[GRAVACAO] Finalizando...");

  if (audioFile) {
    audioFile.seek(0);
    writeWavHeader(audioFile, totalAudioBytes);
    audioFile.flush();
    audioFile.close();
  }

  gravacaoIniciada = false;
  gravacaoPausada = false;

  Serial.print("[GRAVACAO] Arquivo salvo: ");
  Serial.println(AUDIO_FILE_PATH);
  Serial.print("[GRAVACAO] Bytes de audio: ");
  Serial.println(totalAudioBytes);
}

void writeWavHeader(File& file, uint32_t dataSize) {
  byte header[WAV_HEADER_SIZE];

  uint32_t fileSize = dataSize + 36;
  uint32_t byteRate = SAMPLE_RATE * CHANNELS * BYTES_PER_SAMPLE;
  uint16_t blockAlign = CHANNELS * BYTES_PER_SAMPLE;

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF); header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF); header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1; header[21] = 0;
  header[22] = CHANNELS; header[23] = 0;
  header[24] = (byte)(SAMPLE_RATE & 0xFF); header[25] = (byte)((SAMPLE_RATE >> 8) & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >> 16) & 0xFF); header[27] = (byte)((SAMPLE_RATE >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF); header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF); header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(blockAlign & 0xFF); header[33] = (byte)((blockAlign >> 8) & 0xFF);
  header[34] = BITS_PER_SAMPLE; header[35] = 0;
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF); header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF); header[43] = (byte)((dataSize >> 24) & 0xFF);

  file.write(header, WAV_HEADER_SIZE);
}

// ============================================================
// Utilitarios
// ============================================================
uint8_t mapearParaNivel(float valor, float valorPara100) {
  if (valor <= 0.0f) {
    return 0;
  }
  int nivel = (int)((valor / valorPara100) * 100.0f);
  if (nivel < 0) {
    nivel = 0;
  }
  if (nivel > 100) {
    nivel = 100;
  }
  return (uint8_t)nivel;
}

int16_t limitarInt16(float valor) {
  if (valor > 32767.0f) {
    return 32767;
  }
  if (valor < -32768.0f) {
    return -32768;
  }
  return (int16_t)valor;
}
