/*
 * ============================================================
 * Sensor Vibroacustico - INMP441 OU Piezo + OLED
 * Projeto: Localizador Acustico de Vitimas
 *
 * Correção:
 * - Evita conflito ADC driver_ng x I2S legacy no ESP32 Arduino Core 3.x.
 * - O programa inicializa SOMENTE o sensor selecionado.
 *
 * Como usar:
 * - Para testar INMP441: deixe SENSOR_ATIVO = SENSOR_INMP441
 * - Para testar Piezo:   deixe SENSOR_ATIVO = SENSOR_PIEZO
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <limits.h>

#include "interface_estado.h"
#include "interface_display.h"

// ============================================================
// SENSOR ATIVO
// ============================================================
// Use apenas um por vez para evitar conflito entre ADC e I2S legacy.
#define SENSOR_INMP441 1
#define SENSOR_PIEZO   2

#define SENSOR_ATIVO SENSOR_PIEZO
// #define SENSOR_ATIVO SENSOR_PIEZO

// ============================================================
// Imports condicionais
// ============================================================
#if SENSOR_ATIVO == SENSOR_INMP441
  #include <driver/i2s.h>
#endif

// ============================================================
// Display OLED I2C
// ============================================================
// OLED:
// VCC     -> 3V3
// GND     -> GND
// SCK/SCL -> GPIO 22
// SDA     -> GPIO 21
#define OLED_SDA 21
#define OLED_SCL 22

// ============================================================
// Pinos I2S - INMP441
// ============================================================
// INMP441:
// VDD -> 3V3
// GND -> GND
// WS  -> GPIO 25
// SCK -> GPIO 14
// SD  -> GPIO 26
// L/R -> GND
#if SENSOR_ATIVO == SENSOR_INMP441
  #define I2S_WS    25
  #define I2S_SCK   14
  #define I2S_SD    26
  #define I2S_PORT  I2S_NUM_0
  #define BUFFER_LEN 128

  int32_t samplesBuffer[BUFFER_LEN];
#endif

// ============================================================
// Piezo
// ============================================================
// Piezo:
// sinal protegido -> GPIO34
// GND             -> GND
//
// Proteção mínima recomendada:
// Piezo sinal -> resistor série 100k -> GPIO34
// GPIO34 -> resistor 1M -> GND
// Outro lado do piezo -> GND
#if SENSOR_ATIVO == SENSOR_PIEZO
  #define PIEZO_PIN 34
#endif

#define LED_PIN 2

const int PIEZO_THRESHOLD = 120;
const int PIEZO_PARA_100 = 900;

float envelopePiezo = 0.0f;
const float ALPHA_PIEZO = 0.20f;

// ============================================================
// Filtros do INMP441 - 16000 Hz
// ============================================================
#if SENSOR_ATIVO == SENSOR_INMP441
  const float ALPHA_300 = 0.11f;
  const float ALPHA_3000 = 0.54f;

  float grave1_lastY = 0.0f;
  float grave2_lastY = 0.0f;

  float voz_lp1_lastY = 0.0f;
  float voz_lp2_lastY = 0.0f;

  float envelopeVoz = 0.0f;
  const float ALPHA_ENVELOPE = 0.05f;

  const int32_t LIMIAR_VOZ = 1200;
  const float ENVELOPE_PARA_100 = 5000.0f;

  bool vozDetectada = false;
#endif

bool impactoDetectado = false;

// ============================================================
// Estado do sistema
// ============================================================
EstadoSistema estadoAtual = ESTADO_RECEBENDO_SINAIS;

#if SENSOR_ATIVO == SENSOR_INMP441
  ModoOperacao modoAtual = MODO_ACUSTICO;
#else
  ModoOperacao modoAtual = MODO_VIBRACIONAL;
#endif

uint8_t nivelSinalAtual = 0;

unsigned long ultimoRefreshDisplay = 0;
const unsigned long INTERVALO_DISPLAY_MS = 100;

// ============================================================
// Protótipos
// ============================================================
#if SENSOR_ATIVO == SENSOR_INMP441
  void setupI2S();
  uint8_t lerNivelVozINMP441();
  uint8_t mapearEnvelopeVozParaNivel(float envelope);
#endif

#if SENSOR_ATIVO == SENSOR_PIEZO
  uint8_t lerNivelPiezo();
  uint8_t mapearPiezoParaNivel(float envelope);
#endif

uint8_t lerNivelDoSinal();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!displayIniciar()) {
    Serial.println("ERRO: falha ao iniciar display SSD1306");

    while (true) {
      delay(1000);
    }
  }

#if SENSOR_ATIVO == SENSOR_INMP441
  setupI2S();

  Serial.println("Sensor vibroacustico iniciado");
  Serial.println("Modo ativo: ACUSTICO / INMP441");
  Serial.println("INMP441: WS=25, SCK=14, SD=26");
#else
  // Não chamamos analogSetPinAttenuation() para evitar conflito ADC driver_ng.
  // analogRead() simples é suficiente para este teste inicial.
  pinMode(PIEZO_PIN, INPUT);

  Serial.println("Sensor vibroacustico iniciado");
  Serial.println("Modo ativo: VIBRACIONAL / PIEZO");
  Serial.println("Piezo: GPIO34");
#endif

  displayRenderizar(estadoAtual, modoAtual, 0);
}

// ============================================================
// Loop
// ============================================================
void loop() {
  nivelSinalAtual = lerNivelDoSinal();

  unsigned long agora = millis();

  if (agora - ultimoRefreshDisplay >= INTERVALO_DISPLAY_MS) {
    ultimoRefreshDisplay = agora;
    displayRenderizar(estadoAtual, modoAtual, nivelSinalAtual);
  }

  delay(15);
}

// ============================================================
// Leitura por modo
// ============================================================
uint8_t lerNivelDoSinal() {
#if SENSOR_ATIVO == SENSOR_INMP441
  return lerNivelVozINMP441();
#else
  return lerNivelPiezo();
#endif
}

// ============================================================
// INMP441
// ============================================================
#if SENSOR_ATIVO == SENSOR_INMP441

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
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
    Serial.print("ERRO i2s_driver_install: ");
    Serial.println(err);

    while (true) {
      delay(1000);
    }
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);

  if (err != ESP_OK) {
    Serial.print("ERRO i2s_set_pin: ");
    Serial.println(err);

    while (true) {
      delay(1000);
    }
  }

  i2s_zero_dma_buffer(I2S_PORT);
}

uint8_t lerNivelVozINMP441() {
  size_t bytesRead = 0;

  esp_err_t result = i2s_read(
    I2S_PORT,
    samplesBuffer,
    sizeof(samplesBuffer),
    &bytesRead,
    portMAX_DELAY
  );

  if (result != ESP_OK || bytesRead == 0) {
    vozDetectada = false;
    return 0;
  }

  int samplesRead = bytesRead / sizeof(int32_t);

  if (samplesRead == 0) {
    vozDetectada = false;
    return 0;
  }

  int32_t maxVoz = INT32_MIN;
  int32_t minVoz = INT32_MAX;

  for (int i = 0; i < samplesRead; i++) {
    float x = (float)(samplesBuffer[i] >> 16);

    float y_grave1 =
      (ALPHA_300 * x) +
      ((1.0f - ALPHA_300) * grave1_lastY);

    grave1_lastY = y_grave1;

    float y_grave2 =
      (ALPHA_300 * y_grave1) +
      ((1.0f - ALPHA_300) * grave2_lastY);

    grave2_lastY = y_grave2;

    float y_sem_graves = x - y_grave2;

    float y_voz_lp1 =
      (ALPHA_3000 * y_sem_graves) +
      ((1.0f - ALPHA_3000) * voz_lp1_lastY);

    voz_lp1_lastY = y_voz_lp1;

    float y_voz_lp2 =
      (ALPHA_3000 * y_voz_lp1) +
      ((1.0f - ALPHA_3000) * voz_lp2_lastY);

    voz_lp2_lastY = y_voz_lp2;

    int32_t sample_voz = (int32_t)y_voz_lp2;

    if (sample_voz > maxVoz) {
      maxVoz = sample_voz;
    }

    if (sample_voz < minVoz) {
      minVoz = sample_voz;
    }
  }

  float volumeVozBruto = maxVoz - minVoz;

  envelopeVoz =
    (ALPHA_ENVELOPE * volumeVozBruto) +
    ((1.0f - ALPHA_ENVELOPE) * envelopeVoz);

  vozDetectada = envelopeVoz > LIMIAR_VOZ;

  int32_t linhaAlarmeVoz = vozDetectada ? 5000 : 0;
  uint8_t nivel = mapearEnvelopeVozParaNivel(envelopeVoz);

  digitalWrite(LED_PIN, vozDetectada ? HIGH : LOW);

  Serial.print("Modo:ACUSTICO,");
  Serial.print("Base(Zero):0,");
  Serial.print("Deteccao_Voz:");
  Serial.print(linhaAlarmeVoz);
  Serial.print(",");
  Serial.print("EnvelopeVoz:");
  Serial.print(envelopeVoz);
  Serial.print(",");
  Serial.print("Nivel:");
  Serial.println(nivel);

  return nivel;
}

uint8_t mapearEnvelopeVozParaNivel(float envelope) {
  if (envelope <= 0.0f) {
    return 0;
  }

  float normalizado = envelope / ENVELOPE_PARA_100;
  int nivel = (int)(normalizado * 100.0f);

  if (nivel < 0) {
    nivel = 0;
  }

  if (nivel > 100) {
    nivel = 100;
  }

  return (uint8_t)nivel;
}

#endif

// ============================================================
// Piezo
// ============================================================
#if SENSOR_ATIVO == SENSOR_PIEZO

uint8_t lerNivelPiezo() {
  int valor = analogRead(PIEZO_PIN);

  impactoDetectado = valor > PIEZO_THRESHOLD;

  digitalWrite(LED_PIN, impactoDetectado ? HIGH : LOW);

  int valorUtil = valor - PIEZO_THRESHOLD;

  if (valorUtil < 0) {
    valorUtil = 0;
  }

  envelopePiezo =
    (ALPHA_PIEZO * valorUtil) +
    ((1.0f - ALPHA_PIEZO) * envelopePiezo);

  uint8_t nivel = mapearPiezoParaNivel(envelopePiezo);

  Serial.print("Modo:VIBRACIONAL,");
  Serial.print("Piezo:");
  Serial.print(valor);
  Serial.print(",");
  Serial.print("Impacto:");
  Serial.print(impactoDetectado ? 5000 : 0);
  Serial.print(",");
  Serial.print("EnvelopePiezo:");
  Serial.print(envelopePiezo);
  Serial.print(",");
  Serial.print("Nivel:");
  Serial.println(nivel);

  return nivel;
}

uint8_t mapearPiezoParaNivel(float envelope) {
  if (envelope <= 0.0f) {
    return 0;
  }

  float normalizado = envelope / PIEZO_PARA_100;
  int nivel = (int)(normalizado * 100.0f);

  if (nivel < 0) {
    nivel = 0;
  }

  if (nivel > 100) {
    nivel = 100;
  }

  return (uint8_t)nivel;
}

#endif
