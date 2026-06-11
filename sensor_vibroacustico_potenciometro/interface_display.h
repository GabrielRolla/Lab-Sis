#ifndef INTERFACE_DISPLAY_H
#define INTERFACE_DISPLAY_H

#include <Arduino.h>
#include "interface_estado.h"

// Inicializa o display SSD1306.
// Retorna true em caso de sucesso.
bool displayIniciar();

// Renderiza a tela completa de acordo com o estado atual.
// nivelSinal: 0..100.
// Usado principalmente em ESTADO_RECEBENDO_SINAIS.
void displayRenderizar(
  EstadoSistema estado,
  ModoOperacao modo,
  uint8_t nivelSinal,
  uint8_t sensibilidade
);

// Versao completa.
void displayRenderizar(
  EstadoSistema estado,
  ModoOperacao modo,
  uint8_t nivelSinal,
  bool vozDetectada,
  bool impactoDetectado,
  EstadoGravacao estadoGravacao,
  uint8_t sensibilidade
);

#endif
