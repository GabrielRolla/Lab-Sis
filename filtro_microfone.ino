/*
 * ============================================================
 * Detetor Acústico Robusto (Filtros Cascata + Envelope de Energia)
 * Projeto: Localizador Acústico de Vítimas
 * ============================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>

// ----- Pinos I2S -----
#define I2S_WS    25
#define I2S_SCK   14
#define I2S_SD    26
#define I2S_PORT  I2S_NUM_0
#define BUFFER_LEN 128

// ============================================================
// Constantes dos Filtros (16000 Hz)
// ============================================================
const float ALPHA_300 = 0.11f;   // Corta < 300 Hz
const float ALPHA_3000 = 0.54f;  // Corta > 3000 Hz

// Memórias Rota 1 (Maquinário - Graves) - Cascata 2x
float grave1_lastY = 0.0f;
float grave2_lastY = 0.0f;

// Memórias Rota 2 (Voz) - Cascata Passa-Baixa 2x
float voz_lp1_lastY = 0.0f;
float voz_lp2_lastY = 0.0f;

// Envelopes de Suavização (Para ignorar estalos rápidos)
float envelopeGrave = 0.0f;
float envelopeVoz = 0.0f;
const float ALPHA_ENVELOPE = 0.05f; // Quanto menor, mais ignora estalos

// *** LIMIARES DE DETEÇÃO ***
const int32_t LIMIAR_MAQUINA = 2500; 
const int32_t LIMIAR_VOZ = 1200;     

int32_t samplesBuffer[BUFFER_LEN];

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len   = BUFFER_LEN,
    .use_apll      = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk    = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  setupI2S();
}

void loop() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, samplesBuffer, sizeof(samplesBuffer), &bytesRead, portMAX_DELAY);
  int samplesRead = bytesRead / sizeof(int32_t);
  
  if (samplesRead == 0) return;

  int32_t maxGrave = INT32_MIN, minGrave = INT32_MAX;
  int32_t maxVoz = INT32_MIN, minVoz = INT32_MAX;

  for (int i = 0; i < samplesRead; i++) {
    float x = (float)(samplesBuffer[i] >> 16); 

    // ---------------------------------------------------------
    // ROTA 1: MAQUINÁRIO (GRAVES) - Passa-Baixa Agressivo
    // ---------------------------------------------------------
    float y_grave1 = (ALPHA_300 * x) + ((1.0f - ALPHA_300) * grave1_lastY);
    grave1_lastY = y_grave1;
    
    float y_grave2 = (ALPHA_300 * y_grave1) + ((1.0f - ALPHA_300) * grave2_lastY);
    grave2_lastY = y_grave2;

    int32_t sample_grave = (int32_t)y_grave2;
    if (sample_grave > maxGrave) maxGrave = sample_grave;
    if (sample_grave < minGrave) minGrave = sample_grave;

    // ---------------------------------------------------------
    // ROTA 2: VOZ INTELIGÍVEL (MÉDIOS) - Tira os Graves e Limpa Agudos
    // ---------------------------------------------------------
    // Extrai apenas as frequências acima dos graves
    float y_sem_graves = x - y_grave2; 
    
    // Passa-Baixa duplo para remover ruído agudo de chiado (Cascata)
    float y_voz_lp1 = (ALPHA_3000 * y_sem_graves) + ((1.0f - ALPHA_3000) * voz_lp1_lastY);
    voz_lp1_lastY = y_voz_lp1;

    float y_voz_lp2 = (ALPHA_3000 * y_voz_lp1) + ((1.0f - ALPHA_3000) * voz_lp2_lastY);
    voz_lp2_lastY = y_voz_lp2;

    int32_t sample_voz = (int32_t)y_voz_lp2;
    if (sample_voz > maxVoz) maxVoz = sample_voz;
    if (sample_voz < minVoz) minVoz = sample_voz;
  }

  // Calcula o volume físico do pacote
  float volumeGraveBruto = maxGrave - minGrave;
  float volumeVozBruto = maxVoz - minVoz;

  // ---------------------------------------------------------
  // MÁGICA 3: Envelope de Energia (Impede que "Estalos" funcionem)
  // ---------------------------------------------------------
  envelopeGrave = (ALPHA_ENVELOPE * volumeGraveBruto) + ((1.0f - ALPHA_ENVELOPE) * envelopeGrave);
  envelopeVoz = (ALPHA_ENVELOPE * volumeVozBruto) + ((1.0f - ALPHA_ENVELOPE) * envelopeVoz);

  // ---------------------------------------------------------
  // SISTEMA DE ALARME DAS LINHAS
  // ---------------------------------------------------------
  int32_t linhaAlarmeMaquinario = 0;
  int32_t linhaAlarmeVoz = 0;

  if (envelopeGrave > LIMIAR_MAQUINA) {
    linhaAlarmeMaquinario = 5000; 
  }
  if (envelopeVoz > LIMIAR_VOZ) {
    linhaAlarmeVoz = 5000; 
  }

  // Plotagem final
  Serial.print("Base(Zero):0,");
  Serial.print("Deteccao_Maquinario:");
  Serial.print(linhaAlarmeMaquinario);
  Serial.print(",");
  Serial.print("Deteccao_Voz:");
  Serial.println(linhaAlarmeVoz + 6000); // Somado para não sobrepor visualmente

  delay(15); 
}