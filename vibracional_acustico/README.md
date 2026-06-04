# Sensor Vibroacustico com ESP32 + INMP441 + OLED SSD1306

Projeto pronto para teste fisico.

## Componentes

- ESP32 DevKit
- Microfone MEMS Digital INMP441
- Display OLED SSD1306 128x64 I2C
- 3 botoes:
  - ON/OFF
  - MODO
  - INICIAR/PAUSAR

## Ligações

### OLED SSD1306

| OLED | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

### INMP441

| INMP441 | ESP32 |
|---|---|
| VDD | 3V3 |
| GND | GND |
| SCK / BCLK | GPIO 26 |
| WS / LRCL | GPIO 25 |
| SD / DOUT | GPIO 34 |
| L/R | GND |

O código usa `I2S_CHANNEL_FMT_ONLY_LEFT`, por isso o pino L/R do INMP441 deve ir para GND.

### Botões

O código usa `INPUT_PULLUP`.

| Botão | ESP32 |
|---|---|
| ON/OFF | GPIO 32 |
| MODO | GPIO 33 |
| INICIAR/PAUSAR | GPIO 27 |

Ligação de cada botão:

```txt
GPIO ---- botão ---- GND
```

## Bibliotecas

Instalar:

- Adafruit GFX Library
- Adafruit SSD1306

## Ajuste de sensibilidade

No arquivo `sensor_vibroacustico.ino`, ajuste:

```cpp
const float GANHO_MIC = 8.0f;
const float PISO_RUIDO = 180.0f;
const float RMS_PARA_100 = 3500.0f;
```

- Se a barra quase não sobe: aumente `GANHO_MIC`.
- Se a barra fica sempre em 100%: reduza `GANHO_MIC` ou aumente `RMS_PARA_100`.
- Se aparece sinal em silêncio: aumente `PISO_RUIDO`.

## Observação

O modo vibracional está reservado e retorna 0 enquanto o sensor vibracional não for testado.
