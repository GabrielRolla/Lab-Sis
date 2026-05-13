/*
 * ============================================================
 * Detetor Acústico Evoluído (Filtros + Cadência de Voz)
 * Projeto: Localizador de Vítimas em Escombros
 * ============================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>

// ----- Configurações de Pinos (Físico) -----
#define PINO_WS     25
#define PINO_SCK    14
#define PINO_SD     26
#define PORTA_I2S   I2S_NUM_0
#define COMPRIMENTO_BUFFER 128

// ----- Constantes de Processamento de Sinal (DSP) -----
const float ALFA_GRAVES = 0.11f;   // Filtro para frequências < 300 Hz
const float ALFA_AGUDOS = 0.54f;   // Filtro para frequências > 3000 Hz
const float ALFA_ENVELOPE = 0.05f; // Suavização do volume

// ----- Variáveis de Inteligência e Decisão -----
unsigned long tempoUltimaAnalise = 0;
const unsigned long JANELA_MS = 2000; // Analisa o ritmo a cada 2 segundos
int contadorPulsos = 0;               // Quantas "subidas" de som detectamos
bool estadoVozAtiva = false;          // O som está acima do limiar agora?
int32_t LIMIAR_DETECCAO = 1200;       // Ajuste de sensibilidade

// Memórias dos Filtros (Necessárias para o cálculo matemático recursivo)
float grave1_ultimoY = 0, grave2_ultimoY = 0;
float voz_lp1_ultimoY = 0, voz_lp2_ultimoY = 0;
float envelopeEnergia = 0;

int32_t bufferAmostras[COMPRIMENTO_BUFFER];

void configurarI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len   = COMPRIMENTO_BUFFER
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = PINO_SCK,
        .ws_io_num = PINO_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = PINO_SD
    };
    i2s_driver_install(PORTA_I2S, &i2s_config, 0, NULL);
    i2s_set_pin(PORTA_I2S, &pin_config);
}

void setup() {
    Serial.begin(115200);
    configurarI2S();
    tempoUltimaAnalise = millis();
}

// Função que limpa o áudio focando na faixa da voz humana
float filtrarAudio(float amostraEntrada) {
    // 1. Remove Frequências muito baixas (Subtração de Graves)
    float y_grave1 = (ALFA_GRAVES * amostraEntrada) + ((1.0f - ALFA_GRAVES) * grave1_ultimoY);
    grave1_ultimoY = y_grave1;
    float y_grave2 = (ALFA_GRAVES * y_grave1) + ((1.0f - ALFA_GRAVES) * grave2_ultimoY);
    grave2_ultimoY = y_grave2;
    float sinalSemGraves = amostraEntrada - y_grave2;

    // 2. Remove Ruídos Agudos (Filtro Passa-Baixa)
    float y_lp1 = (ALFA_AGUDOS * sinalSemGraves) + ((1.0f - ALFA_AGUDOS) * voz_lp1_ultimoY);
    voz_lp1_ultimoY = y_lp1;
    float y_lp2 = (ALFA_AGUDOS * y_lp1) + ((1.0f - ALFA_AGUDOS) * voz_lp2_ultimoY);
    voz_lp2_ultimoY = y_lp2;

    return y_lp2;
}

void loop() {
    size_t bytesLidos = 0;
    i2s_read(PORTA_I2S, bufferAmostras, sizeof(bufferAmostras), &bytesLidos, portMAX_DELAY);
    int amostrasLidas = bytesLidos / sizeof(int32_t);
    
    if (amostrasLidas == 0) return;

    int32_t maximoVoz = INT32_MIN, minimoVoz = INT32_MAX;

    for (int i = 0; i < amostrasLidas; i++) {
        // Normaliza o áudio do INMP441
        float amostraBruta = (float)(bufferAmostras[i] >> 16);
        float amostraLimpa = filtrarAudio(amostraBruta);
        
        if (amostraLimpa > maximoVoz) maximoVoz = (int32_t)amostraLimpa;
        if (amostraLimpa < minimoVoz) minimoVoz = (int32_t)amostraLimpa;
    }

    // Calcula a amplitude (volume) do pacote atual
    float amplitudeAtual = maximoVoz - minimoVoz;
    
    // Suaviza a variação para criar o "envelope" de energia
    envelopeEnergia = (ALFA_ENVELOPE * amplitudeAtual) + ((1.0f - ALFA_ENVELOPE) * envelopeEnergia);

    // Lógica de Detecção de Pulso (Verifica se o som subiu e desceu)
    if (envelopeEnergia > LIMIAR_DETECCAO && !estadoVozAtiva) {
        contadorPulsos++;
        estadoVozAtiva = true; 
    } else if (envelopeEnergia < (LIMIAR_DETECCAO * 0.8f)) {
        estadoVozAtiva = false;
    }

    // Analisa o padrão rítmico a cada 2 segundos
    if (millis() - tempoUltimaAnalise >= JANELA_MS) {
        int probabilidadeVoz = 0;
        
        // Voz humana típica: 2 a 6 variações por segundo (4 a 12 na nossa janela)
        if (contadorPulsos >= 4 && contadorPulsos <= 12) {
            probabilidadeVoz = 100; 
        } else if (contadorPulsos > 12) {
            probabilidadeVoz = 15; // Provavelmente um ruído de máquina rápido e constante
        }

        // Saída para o Serial Plotter
        Serial.print("Pulsos_no_Tempo:"); Serial.print(contadorPulsos);
        Serial.print(",Probabilidade_Voz:"); Serial.println(probabilidadeVoz);

        // Reseta contadores para o próximo ciclo
        contadorPulsos = 0;
        tempoUltimaAnalise = millis();
    }

    // Visualização em tempo real do sinal filtrado
    Serial.print("Energia_Filtrada:"); Serial.print(envelopeEnergia);
    Serial.print(",Nivel_Corte:"); Serial.println(LIMIAR_DETECCAO);
    
    delay(10);
}
