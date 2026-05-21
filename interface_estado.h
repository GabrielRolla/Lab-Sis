#ifndef INTERFACE_ESTADO_H
#define INTERFACE_ESTADO_H

// Estados do ciclo de operacao da unidade de processamento.
// Corresponde ao diagrama de estados do sensor vibroacustico.
enum EstadoSistema {
  ESTADO_DESLIGADO,
  ESTADO_AGUARDANDO_COMANDO,
  ESTADO_RECEBENDO_SINAIS
};

// Modo de operacao do sensor.
// Acustico: captura via microfone INMP441.
// Vibracional: reservado para sensor de contato/estrutura.
enum ModoOperacao {
  MODO_ACUSTICO,
  MODO_VIBRACIONAL
};

#endif
