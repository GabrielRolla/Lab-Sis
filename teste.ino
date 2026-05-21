/*
 * ============================================================
 * TRANSMISSOR DE ÁUDIO VIA CABO USB (Serial Dump)
 * ============================================================
 * Este código lê o .wav do SD e envia os bytes crus pela Serial.
 */

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  23
#define SPI_MOSI  19

void setup() {
  // Mantemos 115200 para garantir a integridade dos dados
  Serial.begin(115200);
  
  // Dá um tempo para você fechar a Arduino IDE e abrir o script no PC
  delay(5000); 

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("ERRO_SD");
    while(true) { delay(100); }
  }

  File audioFile = SD.open("/gravacao.wav", FILE_READ);
  if (!audioFile) {
    Serial.println("ERRO_ARQUIVO");
    while(true) { delay(100); }
  }

  // 1. Avisa o computador que o áudio vai começar
  Serial.print("---START_OF_WAV---");

  // 2. Lê o arquivo do SD em blocos e envia os bytes brutos pela USB
  uint8_t buffer[256];
  while (audioFile.available()) {
    size_t bytesRead = audioFile.read(buffer, sizeof(buffer));
    Serial.write(buffer, bytesRead); // write() envia binário, print() envia texto!
  }

  audioFile.close();

  // 3. Avisa o computador que o áudio terminou
  Serial.print("---END_OF_WAV---");
}

void loop() {
  // Fica travado após a transmissão
}