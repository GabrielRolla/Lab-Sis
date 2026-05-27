/*
 * ============================================================================
 * GRAVADOR DE ÁUDIO NO SD COM FILTRAGEM DSP EM TEMPO REAL (Foco em Voz Humana)
 * Projeto: Localizador Acústico de Vítimas - Trabalho de Laboratório
 * Baseado no leitura_bruta.ino
 * ============================================================================
 */

/*
- Lógica: o código agora faz a filtragem amostra por amostra (for (int i = 0; i < samplesRead; i++)) dentro do buffer temporário antes de enviar para a função audioFile.write(). Se der algum problema com isso podemos retirar, mas acho que é importante pelo fato do 
- Eficiência de Memória: O filtro calcula tudo usando variáveis flutuantes simples (grave1_lastY, etc.) que guardam apenas o estado anterior. Isso consome quase zero de memória RAM, permitindo filtrar 16 mil amostras por segundo sem engasgar a escrita do SD.
- Validação: acho que um bom teste é gravarem o áudio em um ambiente com um ventilador barulhento ligado e depois colocarem o cartão SD no computador para ouvir o arquivo .wav, vocês notarão que o som do ventilador sumiu ou ficou extremamente abafado, enquanto a voz humana se manterá nítida.
- Pedi pro Gemini colocar uns comentários em outros trechos só pra gente não se perder.
*/

#include <Arduino.h>
#include <driver/i2s.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// ----- MAPEAMENTO DE PINOS DO MICROFONE (I2S) -----
#define I2S_WS     25
#define I2S_SCK    14
#define I2S_SD_PIN 26
#define I2S_PORT   I2S_NUM_0

// ----- MAPEAMENTO DE PINOS DO CARTÃO SD (SPI) -----
#define SD_CS      5
#define SPI_SCK    18
#define SPI_MISO   23
#define SPI_MOSI   19

// ----- CONFIGURAÇÕES DO ÁUDIO E ARQUIVO -----
const int SAMPLE_RATE = 16000;          // Taxa de amostragem (16kHz é ideal para voz humana)
const int RECORD_TIME_SECONDS = 10;     // Tempo fixo de gravação (10 segundos)
const int HEADER_SIZE = 44;             // Tamanho padrão do cabeçalho de um arquivo WAV
// Tamanho total de dados em bytes: Amostragem * 2 bytes (16-bit) * Tempo
const uint32_t EXPECTED_DATA_SIZE = SAMPLE_RATE * 2 * RECORD_TIME_SECONDS;

File audioFile; // Objeto que gerencia o arquivo no cartão SD

// ----- CONFIGURAÇÕES DE FILTRAGEM DIGITAL (DSP) -----
// Constantes matemáticas (Filtros EMA) calculadas para a taxa de 16000Hz
const float ALPHA_300 = 0.11f;   // Constante para isolar frequências abaixo de 300 Hz
const float ALPHA_3000 = 0.54f;  // Constante para isolar frequências acima de 3000 Hz

// Memórias dos filtros: guardam o valor da amostra anterior (essencial para filtros recursivos IIR)
float grave1_lastY = 0.0f;
float grave2_lastY = 0.0f;
float voz_lp1_lastY = 0.0f;
float voz_lp2_lastY = 0.0f;

// ----- BUFFERS DE MEMÓRIA (O colchão de dados) -----
int32_t samplesBuffer[512]; // Recebe os dados brutos de 32-bits vindos do DMA do I2S
int16_t writeBuffer[512];   // Armazena os dados já filtrados e convertidos para 16-bits para salvar no SD
uint32_t totalDataSize = 0; // Contador de bytes já gravados
unsigned long startTime = 0; // Guarda o tempo de início da gravação
int lastSecondPrinted = -1; // Controle para exibir o cronômetro no monitor Serial

// Função de configuração do protocolo I2S do Microfone
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // ESP32 dita o ritmo (Master) e apenas recebe dados (RX)
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,        // O INMP441 exige comunicação em slots de 32-bits
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,         // Gravação em Mono (Apenas canal esquerdo)
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,                                  // 8 buffers interligados eletronicamente
    .dma_buf_len   = 512,                                // Tamanho do buffer (Evita engasgos quando o SD escreve)
    .use_apll      = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk    = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE, // Não usamos pino de saída de áudio
    .data_in_num  = I2S_SD_PIN
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT); // Limpa o lixo de memória do buffer
}

// Função matemática que cria o cabeçalho clássico RIFF/WAVE para o arquivo ser legível no PC
void writeWavHeader(File file, uint32_t dataSize) {
  byte header[HEADER_SIZE];
  uint32_t fileSize = dataSize + 36;
  uint32_t byteRate = SAMPLE_RATE * 2; 

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF); header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF); header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1; header[21] = 0; 
  header[22] = 1; header[23] = 0; 
  header[24] = (byte)(SAMPLE_RATE & 0xFF); header[25] = (byte)((SAMPLE_RATE >> 8) & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >> 16) & 0xFF); header[27] = (byte)((SAMPLE_RATE >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF); header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF); header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = 2; header[33] = 0; 
  header[34] = 16; header[35] = 0; 
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF); header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF); header[43] = (byte)((dataSize >> 24) & 0xFF);
  
  file.write(header, HEADER_SIZE);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Inicializa o barramento SPI definindo a velocidade em 4MHz para evitar ruídos físicos nos fios da protoboard
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("ERRO CRITICO: FALHA no Cartao SD!");
    while(true) { delay(100); }
  }
  
  // Se já existir uma gravação anterior, apaga ela para liberar espaço
  if (SD.exists("/gravacao.wav")) {
    SD.remove("/gravacao.wav");
  }

  // Cria/Abre o arquivo para escrita de dados
  audioFile = SD.open("/gravacao.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println("Erro: Nao foi possivel criar o arquivo.");
    while(true) { delay(100); }
  }

  // Escreve os 44 bytes iniciais do cabeçalho WAV
  writeWavHeader(audioFile, EXPECTED_DATA_SIZE);
  
  // Inicializa o Microfone
  setupI2S();
  Serial.println("\n>>> GRAVANDO COM FILTRO DE VOZ... Fale agora! <<<");
  startTime = millis();
}

void loop() {
  // CRITÉRIO DE PARADA: Se já gravamos os 10 segundos planejados, fecha o arquivo e encerra
  if (totalDataSize >= EXPECTED_DATA_SIZE) {
    audioFile.close();
    Serial.println("=====================================");
    Serial.println("GRAVACAO FILTRADA CONCLUIDA COM SUCESSO!");
    Serial.println("=====================================");
    while(true) { delay(1000); } 
  }

  // Lógica do cronômetro impresso no Monitor Serial
  unsigned long currentMillis = millis();
  int currentSecond = (currentMillis - startTime) / 1000;
  if (currentSecond != lastSecondPrinted && currentSecond <= RECORD_TIME_SECONDS) {
    Serial.print("Tempo de Gravacao: ");
    Serial.print(currentSecond);
    Serial.println("s / 10s");
    lastSecondPrinted = currentSecond;
  }

  // LÊ O MICROFONE: Captura as amostras brutas acumuladas pelo DMA
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, samplesBuffer, sizeof(samplesBuffer), &bytesRead, portMAX_DELAY);
  int samplesRead = bytesRead / sizeof(int32_t); // Descobre quantas amostras de 32-bit foram lidas

  // Trava de segurança para não deixar o arquivo passar do tamanho máximo calculado
  int bytesToWrite = samplesRead * sizeof(int16_t);
  if (totalDataSize + bytesToWrite > EXPECTED_DATA_SIZE) {
    bytesToWrite = EXPECTED_DATA_SIZE - totalDataSize;
    samplesRead = bytesToWrite / sizeof(int16_t);
  }

  // =========================================================================
  // PIPELINE DE PROCESSAMENTO DIGITAL DE SINAIS (DSP) - FILTRAGEM EM TEMPO REAL
  // =========================================================================
  for (int i = 0; i < samplesRead; i++) {
    
    // 1. Extração e Normalização: O INMP441 entrega 24 bits alinhados à esquerda em 32-bits.
    // O shift de 16 bits para a direita reposiciona os dados e os converte para float para podermos aplicar a matemática do filtro.
    float x = (float)(samplesBuffer[i] >> 16); 

    // 2. Rota Passa-Altas (Corta frequências abaixo de 300Hz por subtração):
    // Primeiro calculamos apenas a energia dos graves lentos (abaixo de 300Hz) usando dois estágios de filtro recursivo.
    float y_grave1 = (ALPHA_300 * x) + ((1.0f - ALPHA_300) * grave1_lastY);
    grave1_lastY = y_grave1;
    
    float y_grave2 = (ALPHA_300 * y_grave1) + ((1.0f - ALPHA_300) * grave2_lastY);
    grave2_lastY = y_grave2;

    // Subtração de Sinal: Som Total (x) MENOS o Som Grave (y_grave2) = Som Sem Graves (médios e agudos)
    float y_sem_graves = x - y_grave2; 

    // 3. Rota Passa-Baixas (Corta frequências acima de 3000Hz):
    // Pega o sinal que já está sem os graves e aplica um filtro duplo focado em atenuar chiados rápidos e ruídos agudos.
    float y_voz_lp1 = (ALPHA_3000 * y_sem_graves) + ((1.0f - ALPHA_3000) * voz_lp1_lastY);
    voz_lp1_lastY = y_voz_lp1;

    float y_voz_lp2 = (ALPHA_3000 * y_voz_lp1) + ((1.0f - ALPHA_3000) * voz_lp2_lastY);
    voz_lp2_lastY = y_voz_lp2;

    // 4. Armazenamento: Converte o resultado float filtrado de volta para um inteiro de 16-bits (padrão do arquivo WAV final)
    writeBuffer[i] = (int16_t)y_voz_lp2;
  }
  // =========================================================================

  // Grava o buffer contendo apenas o áudio limpo e filtrado diretamente no Cartão SD
  size_t bytesWritten = audioFile.write((const uint8_t*)writeBuffer, bytesToWrite);
  totalDataSize += bytesWritten; // Atualiza o tamanho do arquivo
}
