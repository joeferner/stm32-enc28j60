
#ifndef _enc28j60_h_
#define _enc28j60_h_

#include <stdint.h>
#include <platform_config.h>
#include <utils/timer.h>

#ifndef MAC_ADDRESS_LENGTH
#  define MAC_ADDRESS_LENGTH 6
#endif

#ifndef ENC28J60_SPI_TIMEOUT
#  define ENC28J60_SPI_TIMEOUT 1000
#endif

typedef struct {
  SPI_HandleTypeDef* spi;
  uint8_t macAddress[MAC_ADDRESS_LENGTH];
  GPIO_TypeDef* csPort;
  uint16_t csPin;
  GPIO_TypeDef* resetPort;
  uint16_t resetPin;

  uint8_t bank;
  int receivedPackets;
  int sentPackets;
  PeriodicTimer watchDogTimer;
} ENC28J60;

HAL_StatusTypeDef ENC28J60_setup(ENC28J60* enc28j60);
void ENC28J60_tick(ENC28J60* enc28j60);
int ENC28J60_send(ENC28J60* enc28j60, const uint8_t* data, uint16_t datalen);
int ENC28J60_receive(ENC28J60* enc28j60, uint8_t* buffer, uint16_t bufsize);

#endif
