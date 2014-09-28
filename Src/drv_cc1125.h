
#ifndef DRV_CC1125_H
#define DRV_CC1125_H

#include "stm32f3xx_hal.h"

/* CC1125 PIN DEFINITIONS */
#define CC1125_SPI_GPIO GPIOB
#define CC1125_SPI SPI2
#define CC1125_NSS_GPIO GPIOB
#define CC1125_NSS_PIN GPIO_PIN_12
#define CC1125_SCK_GPIO GPIOB
#define CC1125_SCK_PIN GPIO_PIN_13
#define CC1125_MISO_GPIO GPIOB
#define CC1125_MISO_PIN GPIO_PIN_14
#define CC1125_MOSI_GPIO GPIOB
#define CC1125_MOSI_PIN GPIO_PIN_15

#define CC1125_GPIO0_GPIO GPIOB
#define CC1125_GPIO0_PIN GPIO_PIN_5
#define CC1125_GPIO2_GPIO GPIOB
#define CC1125_GPIO2_PIN GPIO_PIN_4
#define CC1125_GPIO3_GPIO GPIOB
#define CC1125_GPIO3_PIN GPIO_PIN_3

#define CC1125_RESET_GPIO GPIOA
#define CC1125_RESET_PIN GPIO_PIN_15
/* END CC1125 PIN DEFINITIONS */

/* CC1125 STROBES */
#define STROBE_SRES   0x30
#define STROBE_SCAL   0x33
#define STROBE_SRX    0x34
#define STROBE_STX    0x35
#define STROBE_SIDLE  0x36
#define STROBE_SFRX   0x3A
#define STROBE_SFTX   0x3B
#define STROBE_NOP    0x3D
/* END CC1125 STROBES */

/* CC1125 REGISTERS */
#define REG_IOCFG3           0x0000
#define REG_IOCFG2           0x0001
#define REG_IOCFG1           0x0002
#define REG_IOCFG0           0x0003
#define REG_SYNC3            0x0004
#define REG_SYNC2            0x0005
#define REG_SYNC1            0x0006
#define REG_SYNC0            0x0007
#define REG_SYNC_CFG1        0x0008
#define REG_SYNC_CFG0        0x0009
#define REG_DEVIATION_M      0x000A
#define REG_MODCFG_DEV_E     0x000B
#define REG_DCFILT_CFG       0x000C
#define REG_PREAMBLE_CFG1    0x000D
#define REG_PREAMBLE_CFG0    0x000E
#define REG_FREQ_IF_CFG      0x000F
#define REG_IQIC             0x0010
#define REG_CHAN_BW          0x0011
#define REG_MDMCFG1          0x0012
#define REG_MDMCFG0          0x0013
#define REG_SYMBOL_RATE2     0x0014
#define REG_SYMBOL_RATE1     0x0015
#define REG_SYMBOL_RATE0     0x0016
#define REG_AGC_REF          0x0017
#define REG_AGC_CS_THR       0x0018
#define REG_AGC_GAIN_ADJUST  0x0019
#define REG_AGC_CFG3         0x001A
#define REG_AGC_CFG2         0x001B
#define REG_AGC_CFG1         0x001C
#define REG_AGC_CFG0         0x001D
#define REG_FIFO_CFG         0x001E
#define REG_DEV_ADDR         0x001F
#define REG_SETTLING_CFG     0x0020
#define REG_FS_CFG           0x0021
#define REG_WOR_CFG1         0x0022
#define REG_WOR_CFG0         0x0023
#define REG_WOR_EVENT0_MSB   0x0024
#define REG_WOR_EVENT0_LSB   0x0025
#define REG_PKT_CFG2         0x0026
#define REG_PKT_CFG1         0x0027
#define REG_PKT_CFG0         0x0028
#define REG_RFEND_CFG1       0x0029
#define REG_RFEND_CFG0       0x002A
#define REG_PA_CFG2          0x002B
#define REG_PA_CFG1          0x002C
#define REG_PA_CFG0          0x002D
#define REG_PKT_LEN          0x002E
#define REG_IF_MIX_CFG       0x2F00
#define REG_FREQOFF_CFG      0x2F01
#define REG_TOC_CFG          0x2F02
#define REG_MARC_SPARE       0x2F03
#define REG_ECG_CFG          0x2F04
#define REG_CFM_DATA_CFG     0x2F05
#define REG_EXT_CTRL         0x2F06
#define REG_RCCAL_FINE       0x2F07
#define REG_RCCAL_COARSE     0x2F08
#define REG_RCCAL_OFFSET     0x2F09
#define REG_FREQOFF1         0x2F0A
#define REG_FREQOFF0         0x2F0B
#define REG_FREQ2            0x2F0C
#define REG_FREQ1            0x2F0D
#define REG_FREQ0            0x2F0E
#define REG_IF_ADC2          0x2F0F
#define REG_IF_ADC1          0x2F10
#define REG_IF_ADC0          0x2F11
#define REG_FS_DIG1          0x2F12
#define REG_FS_DIG0          0x2F13
#define REG_FS_CAL3          0x2F14
#define REG_FS_CAL2          0x2F15
#define REG_FS_CAL1          0x2F16
#define REG_FS_CAL0          0x2F17
#define REG_FS_CHP           0x2F18
#define REG_FS_DIVTWO        0x2F19
#define REG_FS_DSM1          0x2F1A
#define REG_FS_DSM0          0x2F1B
#define REG_FS_DVC1          0x2F1C
#define REG_FS_DVC0          0x2F1D
#define REG_FS_LBI           0x2F1E
#define REG_FS_PFD           0x2F1F
#define REG_FS_PRE           0x2F20
#define REG_FS_REG_DIV_CML   0x2F21
#define REG_FS_SPARE         0x2F22
#define REG_FS_VCO4          0x2F23
#define REG_FS_VCO3          0x2F24
#define REG_FS_VCO2          0x2F25
#define REG_FS_VCO1          0x2F26
#define REG_FS_VCO0          0x2F27
#define REG_GBIAS6           0x2F28
#define REG_GBIAS5           0x2F29
#define REG_GBIAS4           0x2F2A
#define REG_GBIAS3           0x2F2B
#define REG_GBIAS2           0x2F2C
#define REG_GBIAS1           0x2F2D
#define REG_GBIAS0           0x2F2E
#define REG_IFAMP            0x2F2F
#define REG_LNA              0x2F30
#define REG_RXMIX            0x2F31
#define REG_XOSC5            0x2F32
#define REG_XOSC4            0x2F33
#define REG_XOSC3            0x2F34
#define REG_XOSC2            0x2F35
#define REG_XOSC1            0x2F36
#define REG_XOSC0            0x2F37
#define REG_ANALOG_SPARE     0x2F38
#define REG_PA_CFG3          0x2F39
#define REG_WOR_TIME1        0x2F64
#define REG_WOR_TIME0        0x2F65
#define REG_WOR_CAPTURE1     0x2F66
#define REG_WOR_CAPTURE0     0x2F67
#define REG_BIST             0x2F68
#define REG_DCFILTOFFSET_I1  0x2F69
#define REG_DCFILTOFFSET_I0  0x2F6A
#define REG_DCFILTOFFSET_Q1  0x2F6B
#define REG_DCFILTOFFSET_Q0  0x2F6C
#define REG_IQIE_I1          0x2F6D
#define REG_IQIE_I0          0x2F6E
#define REG_IQIE_Q1          0x2F6F
#define REG_IQIE_Q0          0x2F70
#define REG_RSSI1            0x2F71
#define REG_RSSI0            0x2F72
#define REG_MARCSTATE        0x2F73
#define REG_LQI_VAL          0x2F74
#define REG_PQT_SYNC_ERR     0x2F75
#define REG_DEM_STATUS       0x2F76
#define REG_FREQOFF_EST1     0x2F77
#define REG_FREQOFF_EST0     0x2F78
#define REG_AGC_GAIN3        0x2F79
#define REG_AGC_GAIN2        0x2F7A
#define REG_AGC_GAIN1        0x2F7B
#define REG_AGC_GAIN0        0x2F7C
#define REG_CFM_RX_DATA_OUT  0x2F7D
#define REG_CFM_TX_DATA_IN   0x2F7E
#define REG_ASK_SOFT_RX_DATA 0x2F7F
#define REG_RNDGEN           0x2F80
#define REG_MAGN2            0x2F81
#define REG_MAGN1            0x2F82
#define REG_MAGN0            0x2F83
#define REG_ANG1             0x2F84
#define REG_ANG0             0x2F85
#define REG_CHFILT_I2        0x2F86
#define REG_CHFILT_I1        0x2F87
#define REG_CHFILT_I0        0x2F88
#define REG_CHFILT_Q2        0x2F89
#define REG_CHFILT_Q1        0x2F8A
#define REG_CHFILT_Q0        0x2F8B
#define REG_GPIO_STATUS      0x2F8C
#define REG_FSCAL_CTRL       0x2F8D
#define REG_PHASE_ADJUST     0x2F8E
#define REG_PARTNUMBER       0x2F8F
#define REG_PARTVERSION      0x2F90
#define REG_SERIAL_STATUS    0x2F91
#define REG_MODEM_STATUS1    0x2F92
#define REG_MODEM_STATUS0    0x2F93
#define REG_MARC_STATUS1     0x2F94
#define REG_MARC_STATUS0     0x2F95
#define REG_PA_IFAMP_TEST    0x2F96
#define REG_FSRF_TEST        0x2F97
#define REG_PRE_TEST         0x2F98
#define REG_PRE_OVR          0x2F99
#define REG_ADC_TEST         0x2F9A
#define REG_DVC_TEST         0x2F9B
#define REG_ATEST            0x2F9C
#define REG_ATEST_LVDS       0x2F9D
#define REG_ATEST_MODE       0x2F9E
#define REG_XOSC_TEST1       0x2F9F
#define REG_XOSC_TEST0       0x2FA0
#define REG_RXFIRST          0x2FD2
#define REG_TXFIRST          0x2FD3
#define REG_RXLAST           0x2FD4
#define REG_TXLAST           0x2FD5
#define REG_NUM_TXBYTES      0x2FD6
#define REG_NUM_RXBYTES      0x2FD7
#define REG_FIFO_NUM_TXBYTES 0x2FD8
#define REG_FIFO_NUM_RXBYTES 0x2FD9
#define EXTENDED_REG_MASK    0x2F
#define REG_FIFO             0x3E
#define READ_BIT_MASK        0x80
#define BURST_BIT_MASK       0x40
/* END CC1125 REGISTERS */

/* CC1125 STATUS BYTE VALUES */
#define STATUS_MASK             0x70 // only these bits should be tested against the status mask
#define STATUS_IDLE             0x00
#define STATUS_RX               0x10
#define STATUS_TX               0x20
#define STATUS_FSTXON           0x30
#define STATUS_CALIBRATE        0x40
#define STATUS_SETTLING         0x50
#define STATUS_RX_FIFO_ERR      0x60
#define STATUS_TX_FIFO_ERR      0x70
/* END CC1125 STATUS BYTE VALUES */

/* CC1125 MARCSTATE VALUES */
// only the ones we need.
#define MARC_IDLE               0x41
/* END CC1125 MARCSTATE VALUES */

/* CC1125 GPIO CONFIGURATIONS */
#define GPIO_RXFIFO_THR         0x00 // high when rxfifo above FIFO_CFG.FIFO_THR
#define GPIO_RXFIFO_THR_PKT     0x01 // high when rxfifo above FIFO_CFG.FIFO_THR, or end of pkt
#define GPIO_TXFIFO_THR         0x02 // assert when txfifo above (127-FIFO_CFG.FIFO_THR), deassert when drained below same
#define GPIO_TXFIFO_THR_PKT     0x03 // assert when txfifo full, deassert when txfifo drained below (127-FIFO_CFG.FIFO_THR)
#define GPIO_RXFIFO_OVERFLOW    0x04 // assert when rxfifo overflowed, deassert when flushed
#define GPIO_TXFIFO_UNDERFLOW   0x05 // assert when txfifo underflowed, deassert when flushed
#define GPIO_PKT_SYNC_RXTX      0x06 // RX: high when pkt receiving, TX: high when sending pkt
#define GPIO_LNA_PA_REG_PD      0x17 // indicates RF operation
#define GPIO_LNA_PD             0x18 // external LNA activation, active low
#define GPIO_PA_PD              0x19 // external PA activation, active low
/* END CC1125 GPIO CONFIGURATIONS */

/* VARIABLES */
extern SPI_HandleTypeDef hspi2;
/* END VARIABLES */

/* SPI BASIC FUNCTIONS */
void cc1125_spi_init(void);
void cc1125_NSS_enable(void);
void cc1125_NSS_disable(void);
uint8_t cc1125_transfer(uint8_t send);
/* END SPI BASIC FUNCTIONS */

/* CC1125 BASIC FUNCTIONS */
uint8_t cc1125WriteStrobe(uint8_t data);
uint8_t cc1125WriteReg(uint16_t addr, uint8_t data);
uint8_t cc1125ReadReg(uint16_t addr);
void cc1125WriteFifo(uint8_t *txData, uint8_t dataLength);
void cc1125ReadFifo(uint8_t *rxData, uint8_t dataLength);
/* END CC1125 BASIC FUNCTIONS */

/* CC1125 ADVANCED FUNCTIONS */
void initcc1125(void);
void cc1125_vco_cal(void);
/* END CC1125 ADVANCED FUNCTIONS */

#endif
