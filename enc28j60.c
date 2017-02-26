/**
 * Inspired from
 * https://github.com/contiki-os/contiki/blob/master/dev/enc28j60/enc28j60.c
 * https://github.com/jcw/ethercard/blob/master/enc28j60.cpp
 */

#include "enc28j60.h"
#include <utils/time.h>
#include <utils/timer.h>

#ifdef ENC28J60_DEBUG
#define ENC28J60_DEBUG_OUT(format, ...) printf("%s:%d: ENC28J60: " format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define ENC28J60_DEBUG_OUT(format, ...)
#endif

#define EIE   0x1b
#define EIR   0x1c
#define ESTAT 0x1d
#define ECON2 0x1e
#define ECON1 0x1f

#define ESTAT_CLKRDY 0x01
#define ESTAT_TXABRT 0x02

#define ECON1_RXEN   0x04
#define ECON1_TXRTS  0x08

#define ECON2_AUTOINC 0x80
#define ECON2_PKTDEC  0x40

#define EIR_TXIF      0x08

#define ERXTX_BANK 0x00

#define ERDPTL 0x00
#define ERDPTH 0x01
#define EWRPTL 0x02
#define EWRPTH 0x03
#define ETXSTL 0x04
#define ETXSTH 0x05
#define ETXNDL 0x06
#define ETXNDH 0x07
#define ERXSTL 0x08
#define ERXSTH 0x09
#define ERXNDL 0x0a
#define ERXNDH 0x0b
#define ERXRDPTL 0x0c
#define ERXRDPTH 0x0d

#define RX_BUF_START 0x0000
#define RX_BUF_END   0x0fff

#define TX_BUF_START 0x1200

/* MACONx registers are in bank 2 */
#define MACONX_BANK 0x02

#define MACON1  0x00
#define MACON3  0x02
#define MACON4  0x03
#define MABBIPG 0x04
#define MAIPGL  0x06
#define MAIPGH  0x07
#define MAMXFLL 0x0a
#define MAMXFLH 0x0b

#define MACON1_TXPAUS 0x08
#define MACON1_RXPAUS 0x04
#define MACON1_MARXEN 0x01

#define MACON3_PADCFG_FULL 0xe0
#define MACON3_TXCRCEN     0x10
#define MACON3_FRMLNEN     0x02
#define MACON3_FULDPX      0x01

#define MAX_MAC_LENGTH 1518

#define MAADRX_BANK 0x03
#define MAADR1 0x04 /* MAADR<47:40> */
#define MAADR2 0x05 /* MAADR<39:32> */
#define MAADR3 0x02 /* MAADR<31:24> */
#define MAADR4 0x03 /* MAADR<23:16> */
#define MAADR5 0x00 /* MAADR<15:8> */
#define MAADR6 0x01 /* MAADR<7:0> */
#define MISTAT 0x0a
#define EREVID 0x12

#define EPKTCNT_BANK 0x01
#define ERXFCON 0x18
#define EPKTCNT 0x19

#define ERXFCON_UCEN  0x80
#define ERXFCON_ANDOR 0x40
#define ERXFCON_CRCEN 0x20
#define ERXFCON_PMEM  0x10
#define ERXFCON_HTEN  0x04
#define ERXFCON_MCEN  0x02
#define ERXFCON_BCEN  0x01

uint8_t _ENC28J60_readRev(ENC28J60* enc28j60);
int _ENC28J60_reset(ENC28J60* enc28j60);
uint8_t _ENC28J60_isMacMiiReg(ENC28J60* enc28j60, uint8_t reg);
uint8_t _ENC28J60_readReg(ENC28J60* enc28j60, uint8_t reg);
void _ENC28J60_writeReg(ENC28J60* enc28j60, uint8_t reg, uint8_t data);
void _ENC28J60_writeReg16(ENC28J60* enc28j60, uint8_t reg, uint16_t data);
void _ENC28J60_setRegBitField(ENC28J60* enc28j60, uint8_t reg, uint8_t mask);
void _ENC28J60_clearRegBitField(ENC28J60* enc28j60, uint8_t reg, uint8_t mask);
void _ENC28J60_setRegBank(ENC28J60* enc28j60, uint8_t new_bank);
void _ENC28J60_writeData(ENC28J60* enc28j60, const uint8_t* data, int datalen);
void _ENC28J60_writeDataByte(ENC28J60* enc28j60, uint8_t byte);
int _ENC28J60_readData(ENC28J60* enc28j60, uint8_t* buf, int len);
uint8_t _ENC28J60_readDataByte(ENC28J60* enc28j60);
void _ENC28J60_softReset(ENC28J60* enc28j60);
void _ENC28J60_spiAssert(ENC28J60* enc28j60);
void _ENC28J60_spiDeassert(ENC28J60* enc28j60);
void _ENC28J60_resetAssert(ENC28J60* enc28j60);
void _ENC28J60_resetDeassert(ENC28J60* enc28j60);
uint8_t _ENC28J60_spiTx(ENC28J60* enc28j60, uint8_t value);

HAL_StatusTypeDef ENC28J60_setup(ENC28J60* enc28j60) {
  enc28j60->bank = ERXTX_BANK;
  enc28j60->receivedPackets = 0;
  enc28j60->sentPackets = 0;
  periodicTimer_setup(&enc28j60->watchDogTimer, 30 * 1000);
  while(_ENC28J60_reset(enc28j60) != 0);
  ENC28J60_DEBUG_OUT("ENC28J60 rev. B%d\n", _ENC28J60_readRev(enc28j60));
  return HAL_OK;
}

uint8_t _ENC28J60_isMacMiiReg(ENC28J60* enc28j60, uint8_t reg) {
  /* MAC or MII register (otherwise, ETH register)? */
  switch (enc28j60->bank) {
  case MACONX_BANK:
    return reg < EIE;
  case MAADRX_BANK:
    return reg <= MAADR2 || reg == MISTAT;
  case ERXTX_BANK:
  case EPKTCNT_BANK:
  default:
    return 0;
  }
}

uint8_t _ENC28J60_readReg(ENC28J60* enc28j60, uint8_t reg) {
  uint8_t r;
  _ENC28J60_spiAssert(enc28j60);
  _ENC28J60_spiTx(enc28j60, 0x00 | (reg & 0x1f));
  if (_ENC28J60_isMacMiiReg(enc28j60, reg)) {
    /* MAC and MII registers require that a dummy byte be read first. */
    _ENC28J60_spiTx(enc28j60, 0x00);
  }
  r = _ENC28J60_spiTx(enc28j60, 0x00);
  _ENC28J60_spiDeassert(enc28j60);
  return r;
}

void _ENC28J60_writeReg16(ENC28J60* enc28j60, uint8_t reg, uint16_t data) {
  _ENC28J60_writeReg(enc28j60, reg, data & 0xff);
  _ENC28J60_writeReg(enc28j60, reg + 1, (data >> 8) & 0xff);
}

void _ENC28J60_writeReg(ENC28J60* enc28j60, uint8_t reg, uint8_t data) {
  _ENC28J60_spiAssert(enc28j60);
  _ENC28J60_spiTx(enc28j60, 0x40 | (reg & 0x1f));
  _ENC28J60_spiTx(enc28j60, data);
  _ENC28J60_spiDeassert(enc28j60);
}

void _ENC28J60_setRegBitField(ENC28J60* enc28j60, uint8_t reg, uint8_t mask) {
  if (_ENC28J60_isMacMiiReg(enc28j60, reg)) {
    _ENC28J60_writeReg(enc28j60, reg, _ENC28J60_readReg(enc28j60, reg) | mask);
  } else {
    _ENC28J60_spiAssert(enc28j60);
    _ENC28J60_spiTx(enc28j60, 0x80 | (reg & 0x1f));
    _ENC28J60_spiTx(enc28j60, mask);
    _ENC28J60_spiDeassert(enc28j60);
  }
}

void _ENC28J60_clearRegBitField(ENC28J60* enc28j60, uint8_t reg, uint8_t mask) {
  if (_ENC28J60_isMacMiiReg(enc28j60, reg)) {
    _ENC28J60_writeReg(enc28j60, reg, _ENC28J60_readReg(enc28j60, reg) & ~mask);
  } else {
    _ENC28J60_spiAssert(enc28j60);
    _ENC28J60_spiTx(enc28j60, 0xa0 | (reg & 0x1f));
    _ENC28J60_spiTx(enc28j60, mask);
    _ENC28J60_spiDeassert(enc28j60);
  }
}

void _ENC28J60_setRegBank(ENC28J60* enc28j60, uint8_t new_bank) {
  _ENC28J60_writeReg(enc28j60, ECON1, (_ENC28J60_readReg(enc28j60, ECON1) & 0xfc) | (new_bank & 0x03));
  enc28j60->bank = new_bank;
}

void _ENC28J60_writeData(ENC28J60* enc28j60, const uint8_t* data, int datalen) {
  int i;
  _ENC28J60_spiAssert(enc28j60);
  /* The Write Buffer Memory (WBM) command is 0 1 1 1 1 0 1 0  */
  _ENC28J60_spiTx(enc28j60, 0x7a);
  for (i = 0; i < datalen; i++) {
    _ENC28J60_spiTx(enc28j60, data[i]);
  }
  _ENC28J60_spiDeassert(enc28j60);
}

void _ENC28J60_writeDataByte(ENC28J60* enc28j60, uint8_t byte) {
  _ENC28J60_writeData(enc28j60, &byte, 1);
}

int _ENC28J60_readData(ENC28J60* enc28j60, uint8_t* buf, int len) {
  int i;
  _ENC28J60_spiAssert(enc28j60);
  /* THe Read Buffer Memory (RBM) command is 0 0 1 1 1 0 1 0 */
  _ENC28J60_spiTx(enc28j60, 0x3a);
  for (i = 0; i < len; i++) {
    buf[i] = _ENC28J60_spiTx(enc28j60, 0x00);
  }
  _ENC28J60_spiDeassert(enc28j60);
  return i;
}

uint8_t _ENC28J60_readDataByte(ENC28J60* enc28j60) {
  uint8_t r;
  _ENC28J60_readData(enc28j60, &r, 1);
  return r;
}

void _ENC28J60_softReset(ENC28J60* enc28j60) {
  ENC28J60_DEBUG_OUT("softReset\n");

  _ENC28J60_spiAssert(enc28j60);
  /* The System Command (soft reset) is 1 1 1 1 1 1 1 1 */
  _ENC28J60_spiTx(enc28j60, 0xff);
  _ENC28J60_spiDeassert(enc28j60);
  enc28j60->bank = ERXTX_BANK;
}

uint8_t _ENC28J60_readRev(ENC28J60* enc28j60) {
  uint8_t rev;
  _ENC28J60_setRegBank(enc28j60, MAADRX_BANK);
  rev = _ENC28J60_readReg(enc28j60, EREVID);
  switch (rev) {
  case 2:
    return 1;
  case 6:
    return 7;
  default:
    return rev;
  }
}

int _ENC28J60_reset(ENC28J60* enc28j60) {
  ENC28J60_DEBUG_OUT("resetting chip\n");

  /*
    6.0 INITIALIZATION

    Before the ENC28J60 can be used to transmit and receive packets,
    certain device settings must be initialized. Depending on the
    application, some configuration options may need to be
    changed. Normally, these tasks may be accomplished once after
    Reset and do not need to be changed thereafter.

    6.1 Receive Buffer

    Before receiving any packets, the receive buffer must be
    initialized by programming the ERXST and ERXND pointers. All
    memory between and including the ERXST and ERXND addresses will be
    dedicated to the receive hardware. It is recommended that the
    ERXST pointer be programmed with an even address.

    Applications expecting large amounts of data and frequent packet
    delivery may wish to allocate most of the memory as the receive
    buffer. Applications that may need to save older packets or have
    several packets ready for transmission should allocate less
    memory.

    When programming the ERXST pointer, the ERXWRPT registers will
    automatically be updated with the same values. The address in
    ERXWRPT will be used as the starting location when the receive
    hardware begins writing received data. For tracking purposes, the
    ERXRDPT registers should additionally be programmed with the same
    value. To program ERXRDPT, the host controller must write to
    ERXRDPTL first, followed by ERXRDPTH.  See Section 7.2.4 “Freeing
    Receive Buffer Space for more information

    6.2 Transmission Buffer

    All memory which is not used by the receive buffer is considered
    the transmission buffer. Data which is to be transmitted should be
    written into any unused space.  After a packet is transmitted,
    however, the hardware will write a seven-byte status vector into
    memory after the last byte in the packet. Therefore, the host
    controller should leave at least seven bytes between each packet
    and the beginning of the receive buffer. No explicit action is
    required to initialize the transmission buffer.

    6.3 Receive Filters

    The appropriate receive filters should be enabled or disabled by
    writing to the ERXFCON register. See Section 8.0 “Receive Filters
    for information on how to configure it.

    6.4 Waiting For OST

    If the initialization procedure is being executed immediately
    following a Power-on Reset, the ESTAT.CLKRDY bit should be polled
    to make certain that enough time has elapsed before proceeding to
    modify the MAC and PHY registers. For more information on the OST,
    see Section 2.2 “Oscillator Start-up Timer.
  */

  _ENC28J60_resetAssert(enc28j60);
  sleep_ms(2);
  _ENC28J60_resetDeassert(enc28j60);
  sleep_ms(2);

  // Not needed? _ENC28J60_softReset(enc28j60);

  /* Workaround for erratum #2. */
  sleep_ms(2);

  /* Wait for OST */
  ENC28J60_DEBUG_OUT("Wait for OST\n");
  uint32_t timeoutTime = HAL_GetTick() + 5000;
  while ((_ENC28J60_readReg(enc28j60, ESTAT) & ESTAT_CLKRDY) == 0) {
    if (HAL_GetTick() > timeoutTime) {
      return 1;
    }
  }
  ENC28J60_DEBUG_OUT("DONE Wait for OST\n");

  _ENC28J60_setRegBank(enc28j60, ERXTX_BANK);
  /* Set up receive buffer */
  _ENC28J60_writeReg16(enc28j60, ERXSTL, RX_BUF_START);
  _ENC28J60_writeReg16(enc28j60, ERXNDL, RX_BUF_END);
  _ENC28J60_writeReg16(enc28j60, ERDPTL, RX_BUF_START);
  _ENC28J60_writeReg16(enc28j60, ERXRDPTL, RX_BUF_END);

  /* Receive filters */
  _ENC28J60_setRegBank(enc28j60, EPKTCNT_BANK);
  _ENC28J60_writeReg(enc28j60, ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

  /*
    6.5 MAC Initialization Settings

    Several of the MAC registers require configuration during
    initialization. This only needs to be done once; the order of
    programming is unimportant.

    1. Set the MARXEN bit in MACON1 to enable the MAC to receive
    frames. If using full duplex, most applications should also set
    TXPAUS and RXPAUS to allow IEEE defined flow control to function.

    2. Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3. Most
    applications should enable automatic padding to at least 60 bytes
    and always append a valid CRC. For convenience, many applications
    may wish to set the FRMLNEN bit as well to enable frame length
    status reporting. The FULDPX bit should be set if the application
    will be connected to a full-duplex configured remote node;
    otherwise, it should be left clear.

    3. Configure the bits in MACON4. For conformance to the IEEE 802.3
    standard, set the DEFER bit.

    4. Program the MAMXFL registers with the maximum frame length to
    be permitted to be received or transmitted. Normal network nodes
    are designed to handle packets that are 1518 bytes or less.

    5. Configure the Back-to-Back Inter-Packet Gap register,
    MABBIPG. Most applications will program this register with 15h
    when Full-Duplex mode is used and 12h when Half-Duplex mode is
    used.

    6. Configure the Non-Back-to-Back Inter-Packet Gap register low
    byte, MAIPGL. Most applications will program this register with
    12h.

    7. If half duplex is used, the Non-Back-to-Back Inter-Packet Gap
    register high byte, MAIPGH, should be programmed. Most
    applications will program this register to 0Ch.

    8. If Half-Duplex mode is used, program the Retransmission and
    Collision Window registers, MACLCON1 and MACLCON2. Most
    applications will not need to change the default Reset values.  If
    the network is spread over exceptionally long cables, the default
    value of MACLCON2 may need to be increased.

    9. Program the local MAC address into the MAADR1:MAADR6 registers.
  */

  _ENC28J60_setRegBank(enc28j60, MACONX_BANK);

  /* Turn on reception and IEEE-defined flow control */
  _ENC28J60_setRegBitField(enc28j60, MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);

  /* Set padding, crc, full duplex */
  _ENC28J60_setRegBitField(
    enc28j60,
    MACON3,
    MACON3_PADCFG_FULL | MACON3_TXCRCEN | MACON3_FULDPX | MACON3_FRMLNEN
  );

  /* Don't modify MACON4 */

  /* Set maximum frame length in MAMXFL */
  _ENC28J60_writeReg16(enc28j60, MAMXFLL, MAX_MAC_LENGTH);

  /* Set back-to-back inter packet gap */
  _ENC28J60_writeReg(enc28j60, MABBIPG, 0x15);

  /* Set non-back-to-back packet gap */
  _ENC28J60_writeReg(enc28j60, MAIPGL, 0x12);

  /* Set MAC address */
  _ENC28J60_setRegBank(enc28j60, MAADRX_BANK);
  _ENC28J60_writeReg(enc28j60, MAADR6, enc28j60->macAddress[5]);
  _ENC28J60_writeReg(enc28j60, MAADR5, enc28j60->macAddress[4]);
  _ENC28J60_writeReg(enc28j60, MAADR4, enc28j60->macAddress[3]);
  _ENC28J60_writeReg(enc28j60, MAADR3, enc28j60->macAddress[2]);
  _ENC28J60_writeReg(enc28j60, MAADR2, enc28j60->macAddress[1]);
  _ENC28J60_writeReg(enc28j60, MAADR1, enc28j60->macAddress[0]);

  /*
    6.6 PHY Initialization Settings

    Depending on the application, bits in three of the PHY module’s
    registers may also require configuration.  The PHCON1.PDPXMD bit
    partially controls the device’s half/full-duplex
    configuration. Normally, this bit is initialized correctly by the
    external circuitry (see Section 2.6 “LED Configuration). If the
    external circuitry is not present or incorrect, however, the host
    controller must program the bit properly. Alternatively, for an
    externally configurable system, the PDPXMD bit may be read and the
    FULDPX bit be programmed to match.

    For proper duplex operation, the PHCON1.PDPXMD bit must also match
    the value of the MACON3.FULDPX bit.

    If using half duplex, the host controller may wish to set the
    PHCON2.HDLDIS bit to prevent automatic loopback of the data which
    is transmitted.  The PHY register, PHLCON, controls the outputs of
    LEDA and LEDB. If an application requires a LED configuration
    other than the default, PHLCON must be altered to match the new
    requirements. The settings for LED operation are discussed in
    Section 2.6 “LED Configuration. The PHLCON register is shown in
    Register 2-2 (page 9).
  */

  /* Don't worry about PHY configuration for now */

  /* Turn on autoincrement for buffer access */
  _ENC28J60_setRegBitField(enc28j60, ECON2, ECON2_AUTOINC);

  /* Turn on reception */
  _ENC28J60_writeReg(enc28j60, ECON1, ECON1_RXEN);
  
  return 0;
}

int ENC28J60_send(ENC28J60* enc28j60, const uint8_t* data, uint16_t datalen) {
  uint16_t dataend;

  /*
    1. Appropriately program the ETXST pointer to point to an unused
       location in memory. It will point to the per packet control
       byte. In the example, it would be programmed to 0120h. It is
       recommended that an even address be used for ETXST.

    2. Use the WBM SPI command to write the per packet control byte,
       the destination address, the source MAC address, the
       type/length and the data payload.

    3. Appropriately program the ETXND pointer. It should point to the
       last byte in the data payload.  In the example, it would be
       programmed to 0156h.

    4. Clear EIR.TXIF, set EIE.TXIE and set EIE.INTIE to enable an
       interrupt when done (if desired).

    5. Start the transmission process by setting
       ECON1.TXRTS.
  */

  _ENC28J60_setRegBank(enc28j60, ERXTX_BANK);
  /* Set up the transmit buffer pointer */
  _ENC28J60_writeReg16(enc28j60, ETXSTL, TX_BUF_START);
  _ENC28J60_writeReg16(enc28j60, EWRPTL, TX_BUF_START);

  /* Write the transmission control register as the first byte of the
     output packet. We write 0x00 to indicate that the default
     configuration (the values in MACON3) will be used.  */
  _ENC28J60_writeDataByte(enc28j60, 0x00); /* MACON3 */

  _ENC28J60_writeData(enc28j60, data, datalen);

  /* Write a pointer to the last data byte. */
  dataend = TX_BUF_START + datalen;
  _ENC28J60_writeReg16(enc28j60, ETXNDL, dataend);

  /* Clear EIR.TXIF */
  _ENC28J60_clearRegBitField(enc28j60, EIR, EIR_TXIF);

  /* Don't care about interrupts for now */

  /* Send the packet */
  _ENC28J60_setRegBitField(enc28j60, ECON1, ECON1_TXRTS);
  uint32_t timeoutTime = HAL_GetTick() + 5000;
  while ((_ENC28J60_readReg(enc28j60, ECON1) & ECON1_TXRTS) > 0) {
    if (HAL_GetTick() > timeoutTime) {
      ENC28J60_DEBUG_OUT("timeout sending packet\n");
      return 0;
    }
  }

#ifdef ENC28J60_DEBUG
  if ((_ENC28J60_readReg(enc28j60, ESTAT) & ESTAT_TXABRT) != 0) {
    uint16_t erdpt;
    uint8_t tsv[7];
    erdpt = (_ENC28J60_readReg(enc28j60, ERDPTH) << 8) | _ENC28J60_readReg(enc28j60, ERDPTL);
    _ENC28J60_writeReg16(enc28j60, ERDPTL, dataend + 1);
    _ENC28J60_readData(enc28j60, tsv, sizeof(tsv));
    _ENC28J60_writeReg16(enc28j60, ERDPTL, erdpt);
    ENC28J60_DEBUG_OUT("tx err: %d: %02x:%02x:%02x:%02x:%02x:%02x\n"
                       "                  tsv: %02x%02x%02x%02x%02x%02x%02x\n", datalen,
                       data[0], data[1], data[2],
                       data[3], data[4], data[5],
                       tsv[6], tsv[5], tsv[4], tsv[3], tsv[2], tsv[1], tsv[0]);
  } else {
    ENC28J60_DEBUG_OUT("tx: %d: %02x:%02x:%02x:%02x:%02x:%02x\n", datalen,
                       data[0], data[1], data[2],
                       data[3], data[4], data[5]);
  }
#endif

  enc28j60->sentPackets++;
  ENC28J60_DEBUG_OUT("sentPackets %d\n", enc28j60->sentPackets);
  return datalen;
}

int ENC28J60_receive(ENC28J60* enc28j60, uint8_t* buffer, uint16_t bufsize) {
  int n, len, next, err;

  uint8_t nxtpkt[2];
  uint8_t status[2];
  uint8_t length[2];

  err = 0;

  _ENC28J60_setRegBank(enc28j60, EPKTCNT_BANK);
  n = _ENC28J60_readReg(enc28j60, EPKTCNT);

  if (n == 0) {
    return 0;
  }

  ENC28J60_DEBUG_OUT("EPKTCNT 0x%02x\n", n);

  _ENC28J60_setRegBank(enc28j60, ERXTX_BANK);
  /* Read the next packet pointer */
  nxtpkt[0] = _ENC28J60_readDataByte(enc28j60);
  nxtpkt[1] = _ENC28J60_readDataByte(enc28j60);

  ENC28J60_DEBUG_OUT("nxtpkt 0x%02x%02x\n", nxtpkt[1], nxtpkt[0]);

  length[0] = _ENC28J60_readDataByte(enc28j60);
  length[1] = _ENC28J60_readDataByte(enc28j60);

  ENC28J60_DEBUG_OUT("length 0x%02x%02x\n", length[1], length[0]);

  status[0] = _ENC28J60_readDataByte(enc28j60);
  status[1] = _ENC28J60_readDataByte(enc28j60);

  /* This statement is just to avoid a compiler warning: */
  status[0] = status[0];
  ENC28J60_DEBUG_OUT("status 0x%02x%02x\n", status[1], status[0]);

  len = (length[1] << 8) + length[0];
  if (bufsize >= len) {
    _ENC28J60_readData(enc28j60, buffer, len);
  } else {
    uint16_t i;

    err = 1;

    /* flush rx fifo */
    for (i = 0; i < len; i++) {
      _ENC28J60_readDataByte(enc28j60);
    }
  }

  /* Read an additional byte at odd lengths, to avoid FIFO corruption */
  if ((len % 2) != 0) {
    _ENC28J60_readDataByte(enc28j60);
  }

  /* Errata #14 */
  next = (nxtpkt[1] << 8) + nxtpkt[0];
  if (next == RX_BUF_START) {
    next = RX_BUF_END;
  } else {
    next = next - 1;
  }
  _ENC28J60_writeReg16(enc28j60, ERXRDPTL, next);

  _ENC28J60_setRegBitField(enc28j60, ECON2, ECON2_PKTDEC);

  if (err) {
    ENC28J60_DEBUG_OUT("rx err: flushed %d\n", len);
    return 0;
  }
  ENC28J60_DEBUG_OUT(
    "rx: %d: %02x:%02x:%02x:%02x:%02x:%02x\n",
    len,
    buffer[0], buffer[1], buffer[2],
    buffer[3], buffer[4], buffer[5]
  );

  enc28j60->receivedPackets++;
  ENC28J60_DEBUG_OUT("receivedPackets %d\n", enc28j60->receivedPackets);
  return len;
}

void ENC28J60_tick(ENC28J60* enc28j60) {
  if (periodicTimer_hasElapsed(&enc28j60->watchDogTimer)) {
    ENC28J60_DEBUG_OUT(
      "test received_packet %d > sentPackets %d\n",
      enc28j60->receivedPackets,
      enc28j60->sentPackets
    );
    if (enc28j60->receivedPackets <= enc28j60->sentPackets) {
      ENC28J60_DEBUG_OUT("resetting chip\n");
      _ENC28J60_reset(enc28j60);
    }
    enc28j60->receivedPackets = 0;
    enc28j60->sentPackets = 0;
  }
}

void _ENC28J60_spiAssert(ENC28J60* enc28j60) {
  HAL_GPIO_WritePin(enc28j60->csPort, enc28j60->csPin, GPIO_PIN_RESET);
}

void _ENC28J60_spiDeassert(ENC28J60* enc28j60) {
  HAL_GPIO_WritePin(enc28j60->csPort, enc28j60->csPin, GPIO_PIN_SET);
}

uint8_t _ENC28J60_spiTx(ENC28J60* enc28j60, uint8_t value) {
  uint8_t tx[1];
  uint8_t rx[1];
  tx[0] = value;
  HAL_SPI_TransmitReceive(enc28j60->spi, tx, rx, 1, ENC28J60_SPI_TIMEOUT);
  return rx[0];
}

void _ENC28J60_resetAssert(ENC28J60* enc28j60) {
  HAL_GPIO_WritePin(enc28j60->resetPort, enc28j60->resetPin, GPIO_PIN_RESET);
}

void _ENC28J60_resetDeassert(ENC28J60* enc28j60) {
  HAL_GPIO_WritePin(enc28j60->resetPort, enc28j60->resetPin, GPIO_PIN_SET);
}
