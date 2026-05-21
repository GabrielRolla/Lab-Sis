#include "interface_display.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Configuracao do display ----------
static const uint8_t LARGURA = 128;
static const uint8_t ALTURA  = 64;
static const uint8_t ENDERECO_I2C = 0x3C;

// Display OLED SSD1306 128x64 via I2C
static Adafruit_SSD1306 oled(LARGURA, ALTURA, &Wire, -1);

// ---------- Layout das barras ----------
static const uint8_t N_BARRAS = 16;
static const uint8_t BARRA_LARGURA = 6;
static const uint8_t BARRA_ESPACAMENTO = 2;

static const uint8_t BARRAS_AREA_Y = 30;
static const uint8_t BARRAS_AREA_ALTURA = 30;

// ---------- Prototipos internos ----------
static void telaDesligado();
static void telaAguardandoComando(ModoOperacao modo);
static void telaRecebendoSinais(ModoOperacao modo, uint8_t nivel);

static const char* obterNomeModo(ModoOperacao modo);
static void desenharCabecalho(const char* titulo, ModoOperacao modo);
static void desenharBarrasIntensidade(uint8_t nivel);

// ---------- API publica ----------

bool displayIniciar() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, ENDERECO_I2C)) {
    return false;
  }

  oled.clearDisplay();
  oled.display();

  return true;
}

void displayRenderizar(
  EstadoSistema estado,
  ModoOperacao modo,
  uint8_t nivelSinal
) {
  oled.clearDisplay();

  switch (estado) {
    case ESTADO_DESLIGADO:
      telaDesligado();
      break;

    case ESTADO_AGUARDANDO_COMANDO:
      telaAguardandoComando(modo);
      break;

    case ESTADO_RECEBENDO_SINAIS:
      telaRecebendoSinais(modo, nivelSinal);
      break;
  }

  oled.display();
}

// ---------- Helpers internos ----------

static const char* obterNomeModo(ModoOperacao modo) {
  if (modo == MODO_ACUSTICO) {
    return "ACUST";
  }

  return "VIBRA";
}

static void desenharCabecalho(const char* titulo, ModoOperacao modo) {
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Titulo do estado
  oled.setCursor(0, 0);
  oled.print(titulo);

  // Modo atual alinhado a direita
  const char* nomeModo = obterNomeModo(modo);
  oled.setCursor(LARGURA - 30, 0);
  oled.print(nomeModo);

  // Linha separadora
  oled.drawLine(0, 10, LARGURA - 1, 10, SSD1306_WHITE);
}

static void desenharBarrasIntensidade(uint8_t nivel) {
  if (nivel > 100) {
    nivel = 100;
  }

  uint8_t barrasAcesas = (uint16_t)nivel * N_BARRAS / 100;

  if (barrasAcesas > N_BARRAS) {
    barrasAcesas = N_BARRAS;
  }

  for (uint8_t i = 0; i < N_BARRAS; i++) {
    uint8_t x = i * (BARRA_LARGURA + BARRA_ESPACAMENTO);

    // Altura crescente para criar efeito de medidor/VU.
    uint8_t alturaMax = BARRAS_AREA_ALTURA * (i + 1) / N_BARRAS;

    if (alturaMax < 4) {
      alturaMax = 4;
    }

    uint8_t y = BARRAS_AREA_Y + (BARRAS_AREA_ALTURA - alturaMax);

    if (i < barrasAcesas) {
      oled.fillRect(
        x,
        y,
        BARRA_LARGURA,
        alturaMax,
        SSD1306_WHITE
      );
    } else {
      // Referencia visual da barra apagada.
      oled.drawRect(
        x,
        BARRAS_AREA_Y + BARRAS_AREA_ALTURA - 2,
        BARRA_LARGURA,
        2,
        SSD1306_WHITE
      );
    }
  }
}

// ---------- Telas ----------

static void telaDesligado() {
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(28, 20);
  oled.print("SISTEMA");

  oled.setTextSize(2);
  oled.setCursor(20, 32);
  oled.print("DESLIGADO");
}

static void telaAguardandoComando(ModoOperacao modo) {
  desenharCabecalho("LIGADO", modo);

  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(8, 22);
  oled.print("Aguardando comando");

  oled.setCursor(0, 40);
  oled.print("[INI] iniciar leitura");

  oled.setCursor(0, 52);
  oled.print("[MOD] trocar modo");
}

static void telaRecebendoSinais(ModoOperacao modo, uint8_t nivel) {
  desenharCabecalho("LENDO", modo);

  if (nivel > 100) {
    nivel = 100;
  }

  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 14);
  oled.print("Sinal:");

  oled.setCursor(40, 14);

  if (nivel < 10) {
    oled.print("  ");
  } else if (nivel < 100) {
    oled.print(" ");
  }

  oled.print(nivel);
  oled.print("%");

  desenharBarrasIntensidade(nivel);
}
