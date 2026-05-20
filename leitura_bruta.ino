/*
 * ============================================================
 * GRAVADOR DE ÁUDIO NO SD (Equilíbrio de Buffer / 4MHz SPI)
 * ============================================================
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define I2S_WS    25
#define I2S_SCK   14
#define I2S_SD_PIN 26
#define I2S_PORT  I2S_NUM_0

#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  23
#define SPI_MOSI  19

const int SAMPLE_RATE = 16000;
const int RECORD_TIME_SECONDS = 10; 
const int HEADER_SIZE = 44;
const uint32_t EXPECTED_DATA_SIZE = SAMPLE_RATE * 2 * RECORD_TIME_SECONDS;

File audioFile;

// ------------------------------------------------------------
// O MEIO-TERMO SEGURO: Buffer de 512 em vez de 256 (Ajuda no chiado sem travar o ESP)
// ------------------------------------------------------------
int32_t samplesBuffer[512];
int16_t writeBuffer[512]; 
uint32_t totalDataSize = 0;
unsigned long startTime = 0;
int lastSecondPrinted = -1;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len   = 512, // Dobramos para dar mais fôlego contra as travadas do SD
    .use_apll      = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk    = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD_PIN
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

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
  
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  // Mantivemos em 4000000 (4MHz) porque é o que suporta os fios da sua protoboard!
  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("ERRO CRITICO: FALHA no Cartao SD!");
    while(true) { delay(100); }
  }
  
  if (SD.exists("/gravacao.wav")) {
    SD.remove("/gravacao.wav");
  }

  audioFile = SD.open("/gravacao.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println("Erro: Nao foi possivel criar o arquivo.");
    while(true) { delay(100); }
  }

  writeWavHeader(audioFile, EXPECTED_DATA_SIZE);
  
  setupI2S();
  Serial.println("\n>>> GRAVANDO... Fale agora! <<<");
  startTime = millis();
}

void loop() {
  if (totalDataSize >= EXPECTED_DATA_SIZE) {
    audioFile.close();
    Serial.println("=====================================");
    Serial.println("GRAVACAO CONCLUIDA COM SUCESSO!");
    Serial.println("=====================================");
    while(true) { delay(1000); } 
  }

  unsigned long currentMillis = millis();
  int currentSecond = (currentMillis - startTime) / 1000;

  if (currentSecond != lastSecondPrinted && currentSecond <= RECORD_TIME_SECONDS) {
    Serial.print("Tempo: ");
    Serial.print(currentSecond);
    Serial.println(" / 10s");
    lastSecondPrinted = currentSecond;
  }

  size_t bytesRead = 0;
  i2s_read(I2S_PORT, samplesBuffer, sizeof(samplesBuffer), &bytesRead, portMAX_DELAY);
  int samplesRead = bytesRead / sizeof(int32_t);

  int bytesToWrite = samplesRead * sizeof(int16_t);
  if (totalDataSize + bytesToWrite > EXPECTED_DATA_SIZE) {
    bytesToWrite = EXPECTED_DATA_SIZE - totalDataSize;
    samplesRead = bytesToWrite / sizeof(int16_t);
  }

  for (int i = 0; i < samplesRead; i++) {
    writeBuffer[i] = (int16_t)(samplesBuffer[i] >> 16);
  }

  size_t bytesWritten = audioFile.write((const uint8_t*)writeBuffer, bytesToWrite);
  totalDataSize += bytesWritten;
}