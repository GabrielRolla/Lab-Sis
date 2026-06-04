#ifndef INTERFACE_ESTADO_H
#define INTERFACE_ESTADO_H

// Estados do ciclo de operacao da unidade de processamento.
enum EstadoSistema {
  ESTADO_DESLIGADO,
  ESTADO_AGUARDANDO_COMANDO,
  ESTADO_RECEBENDO_SINAIS
};

// Modo de operacao do sensor.
enum ModoOperacao {
  MODO_ACUSTICO,
  MODO_VIBRACIONAL
};

// Estado da gravacao no SD.
enum EstadoGravacao {
  GRAVACAO_PARADA,
  GRAVACAO_GRAVANDO,
  GRAVACAO_PAUSADA
};

#endif
