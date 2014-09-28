
#include "drv_cc1125.h"

// TODO: move all spi.c stuff here
// TODO: replace magic numbers with #defines
// TODO: general cleanup
// TODO: allow multiple RF configurations
// TODO: link setup to hwtype

/* INIT FUNCTION */

void cc1125_spi_init(void)
{
    HAL_GPIO_WritePin(CC1125_NSS_GPIO, CC1125_NSS_PIN, GPIO_PIN_SET); // set NSS disabled for now
    HAL_GPIO_WritePin(CC1125_RESET_GPIO, CC1125_RESET_PIN, GPIO_PIN_RESET); // Reset is active low. Keep the chip in reset by default
}

/* END INIT FUNCTION */

/* BASIC FUNCTIONS */

void cc1125_NSS_enable(void)
{
    HAL_GPIO_WritePin(CC1125_NSS_GPIO, CC1125_NSS_PIN, GPIO_PIN_RESET);
    while(HAL_GPIO_ReadPin(CC1125_MISO_GPIO, CC1125_MISO_PIN)); // wait for MISO to go low, indicates ready TODO: timeout handler
}

void cc1125_NSS_disable(void)
{
    HAL_GPIO_WritePin(CC1125_NSS_GPIO, CC1125_NSS_PIN, GPIO_PIN_SET);
}

uint8_t cc1125_transfer(uint8_t send)
{
    uint8_t ret;
    HAL_SPI_TransmitReceive(&hspi2, &send, &ret, 1, HAL_MAX_DELAY); // TODO: set an appropriate timeout and handler
    return ret;
}

/* END BASIC FUNCTIONS */

/* CC1125 BASIC FUNCTIONS */

uint8_t cc1125WriteStrobe(uint8_t data)
{
    uint8_t ret;
    cc1125_NSS_enable();
    ret = cc1125_transfer(data); // TODO: fix function names to be similar.
    cc1125_NSS_disable();
    return ret;
}

uint8_t cc1125WriteReg(uint16_t addr, uint8_t data)
{
    // NOTE: cc1125 should be in IDLE state to write registers.
    
    uint8_t ret;
    uint8_t tempExt  = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);

    cc1125_NSS_enable();

    /* Decide what register space is accessed */  
    if (tempExt == EXTENDED_REG_MASK)
    {
        cc1125_transfer(tempExt);
    }
    cc1125_transfer(tempAddr);
    ret = cc1125_transfer(data); // status byte always returned when data is written.

    cc1125_NSS_disable();
    
    return ret;
}

uint8_t cc1125ReadReg(uint16_t addr)
{
    uint8_t tempExt  = (uint8_t)(addr>>8);
    uint8_t tempAddr = (uint8_t)(addr & 0x00FF);
    uint8_t ret;

    cc1125_NSS_enable();

    /* Decide what register space is accessed */  
    if ((tempExt == EXTENDED_REG_MASK) || (tempExt == REG_FIFO))
    {
        cc1125_transfer((0x80 | (0x7F & tempExt))); //make sure read bit is specified TODO: cleanup
        cc1125_transfer(tempAddr);
    } else {
        cc1125_transfer((0x80 | (0x7F & tempAddr))); //make sure read bit is specified
    }
    ret = cc1125_transfer(0x00);
    cc1125_NSS_disable();

    return ret;
}

void cc1125WriteFifo(uint8_t *txData, uint8_t dataLength)
{
    cc1125WriteStrobe(STROBE_SIDLE); // enter idle state (txfifo reset only possible in this mode)
    cc1125WriteStrobe(STROBE_SFTX); // flush txfifo
    //  cc1125WriteReg(REG_TXFIRST, 0x00); // reset TXFIRST pointer
    //  cc1125WriteReg(REG_TXLAST, dataLength); // reset TXLAST pointer

    cc1125_NSS_enable();
    cc1125_transfer(BURST_BIT_MASK | REG_FIFO); // 7=write, 6=burst, 5-0=0x3F txfifo address
    cc1125_transfer(dataLength);
    for(uint8_t i = 0; i < dataLength; i++)
    {
        cc1125_transfer(txData[i]);
    }
    cc1125_NSS_disable();
}

void cc1125ReadFifo(uint8_t *rxData, uint8_t dataLength)
{
    cc1125_NSS_enable();
    cc1125_transfer(READ_BIT_MASK | BURST_BIT_MASK | REG_FIFO); // 7=read, 6=burst, 5-0=0x3F rxfifo address
    cc1125_transfer(0x00); // first byte is length, should already be known TODO: this should not be here?
    for(uint8_t i = 0; i < dataLength; i++)
    {
        rxData[i] = cc1125_transfer(0x00);
    }
    cc1125_NSS_disable();

    cc1125WriteStrobe(STROBE_SIDLE); // enter idle state (rxfifo reset only possible in this mode)
    cc1125WriteStrobe(STROBE_SFRX); // flush RX fifo
    cc1125WriteReg(REG_RXFIRST, 0x00); // reset RXFIRST pointer
    cc1125WriteReg(REG_RXLAST, 0x00); // reset RXLAST pointer
}

/* END CC1125 BASIC FUNCTIONS */

/* CC1125 ADVANCED FUNCTIONS */

void initcc1125(void) {
    HAL_GPIO_WritePin(CC1125_RESET_GPIO, CC1125_RESET_PIN, GPIO_PIN_SET); // take cc1125 out of reset state
    HAL_Delay(50);
    cc1125WriteStrobe(STROBE_SRES); // reset chip
    HAL_Delay(50);

    // TODO: this, of course.
    // NOTE: see cc1125 user guide, section 3.4 page 18 for GPIO info and settings.
    cc1125WriteReg(0x0000,0xB0); // GPIO3 IO Pin Configuration (IOCFG3)
    cc1125WriteReg(0x0001,0x17); // GPIO2 IO Pin Configuration (IOCFG2) tie to internal PA/LNA, active low LNA_PA_REG_PD
    cc1125WriteReg(0x0002,0xB0); // GPIO1 IO Pin Configuration (IOCFG1) this is also MISO
    cc1125WriteReg(0x0003,0x06); // GPIO0 IO Pin Configuration (IOCFG0) PKT_SYNC_RXTX 
    //  cc1125WriteReg(0x0004,0xD3); // Sync Word Configuration [31:24] (SYNC3)
    //  cc1125WriteReg(0x0005,0x91); // Sync Word Configuration [23:16] (SYNC2)
    //  cc1125WriteReg(0x0006,0xD3); // Sync Word Configuration [15:8] (SYNC1)
    //  cc1125WriteReg(0x0007,0x91); // Sync Word Configuration [7:0] (SYNC0)
    //  cc1125WriteReg(0x0008,0x0B); // Sync Word Detection Configuration Reg. 1 (SYNC_CFG1)
    cc1125WriteReg(0x0008,0x08); // (SYNC_CFG1) ---------------------------------------------------------------------------------------
    cc1125WriteReg(0x000A,0x48); // (DEVIATION_M) -------------------------------------------------------------------------------------
    cc1125WriteReg(0x000B,0x2C); // (MODCFG_DEV_E)  03=2fsk 0B=2gfsk 23=4fsk 2b=4gfsk ----------------------------------------------
    cc1125WriteReg(0x000C,0x1C); // Digital DC Removal Configuration (DCFILT_CFG)
    //  cc1125WriteReg(0x000D,0x18); // Preamble Length Configuration Reg. 1 (PREAMBLE_CFG1)
    cc1125WriteReg(0x0010,0xC6); // Digital Image Channel Compensation Configuration (IQIC)
    //  cc1125WriteReg(0x0011,0x08); // Channel Filter Configuration (CHAN_BW)
    cc1125WriteReg(0x0011,0x0A); // 20kHz RXBW (CHAN_BW) ---------------------------------------------------------------------------------------
    cc1125WriteReg(0x0013,0x05); // General Modem Parameter Configuration Reg. 0 (MDMCFG0)
    cc1125WriteReg(0x0014,0x73); // 9.6ksps (SYMBOL_RATE2) ---------------------------------------------------------------------------
    cc1125WriteReg(0x0017,0x20); // AGC Reference Level Configuration (AGC_REF)
    cc1125WriteReg(0x0018,0x19); // Carrier Sense Threshold Configuration (AGC_CS_THR)
    cc1125WriteReg(0x001C,0xA9); // Automatic Gain Control Configuration Reg. 1 (AGC_CFG1)
    cc1125WriteReg(0x001D,0xCF); // Automatic Gain Control Configuration Reg. 0 (AGC_CFG0)
    cc1125WriteReg(0x001E,0x00); // FIFO Configuration (FIFO_CFG)
    cc1125WriteReg(0x0020,0x03); // Frequency Synthesizer Calibration and Settling Con.. (SETTLING_CFG)
    cc1125WriteReg(0x0021,0x14); // Frequency Synthesizer Configuration (FS_CFG)
    cc1125WriteReg(0x0026,0x00); // Packet Configuration Reg. 2 (PKT_CFG2)
    cc1125WriteReg(0x0028,0x20); // Packet Configuration Reg. 0 (PKT_CFG0)
    cc1125WriteReg(0x002E,0xFF); // Packet Length Configuration (PKT_LEN)
    cc1125WriteReg(0x2F00,0x00); // IF Mix Configuration (IF_MIX_CFG)
    cc1125WriteReg(0x2F01,0x22); // Frequency Offset Correction Configuration (FREQOFF_CFG)
    cc1125WriteReg(0x2F0C,0x6C); // Frequency Configuration [23:16] (FREQ2)     434
    cc1125WriteReg(0x2F0D,0x80); // Frequency Configuration [15:8] (FREQ1)      434
    //  cc1125WriteReg(0x2F0C,0x6E); // Frequency Configuration [23:16] (FREQ2)     440
    //  cc1125WriteReg(0x2F0D,0x00); // Frequency Configuration [15:8] (FREQ1)      440
    cc1125WriteReg(0x2F12,0x00); // Frequency Synthesizer Digital Reg. 1 (FS_DIG1)
    cc1125WriteReg(0x2F13,0x5F); // Frequency Synthesizer Digital Reg. 0 (FS_DIG0)
    cc1125WriteReg(0x2F16,0x40); // Frequency Synthesizer Calibration Reg. 1 (FS_CAL1)
    cc1125WriteReg(0x2F17,0x0E); // Frequency Synthesizer Calibration Reg. 0 (FS_CAL0)
    cc1125WriteReg(0x2F19,0x03); // Frequency Synthesizer Divide by 2 (FS_DIVTWO)
    cc1125WriteReg(0x2F1B,0x33); // FS Digital Synthesizer Module Configuration Reg. 0 (FS_DSM0)
    cc1125WriteReg(0x2F1D,0x17); // Frequency Synthesizer Divider Chain Configuration .. (FS_DVC0)
    cc1125WriteReg(0x2F1F,0x50); // Frequency Synthesizer Phase Frequency Detector Con.. (FS_PFD)
    cc1125WriteReg(0x2F20,0x6E); // Frequency Synthesizer Prescaler Configuration (FS_PRE)
    cc1125WriteReg(0x2F21,0x14); // Frequency Synthesizer Divider Regulator Configurat.. (FS_REG_DIV_CML)
    cc1125WriteReg(0x2F22,0xAC); // Frequency Synthesizer Spare (FS_SPARE)
    cc1125WriteReg(0x2F27,0xB4); // FS Voltage Controlled Oscillator Configuration Reg.. (FS_VCO0)
    cc1125WriteReg(0x2F32,0x0E); // Crystal Oscillator Configuration Reg. 5 (XOSC5)
    cc1125WriteReg(0x2F36,0x03); // Crystal Oscillator Configuration Reg. 1 (XOSC1)
  
    cc1125_vco_cal();
}

void cc1125_vco_cal(void)
{
    // TODO: check for timeouts, indicate error
    // NOTE: this is needed on all cc112x.
  
    uint8_t original_fs_cal2;
    uint8_t calResults_for_vcdac_start_high[3];
    uint8_t calResults_for_vcdac_start_mid[3];

    cc1125WriteReg(REG_FS_VCO2, 0x00); // FS_VCO2 set to 0x00
    original_fs_cal2 = cc1125ReadReg(REG_FS_CAL2); // read FS_CAL2 to original_fs_cal2
    cc1125WriteReg(REG_FS_CAL2, original_fs_cal2+2); // set FS_CAL2 higher
    cc1125WriteStrobe(STROBE_SCAL); // strobe SCAL to start calibration
    while(cc1125ReadReg(REG_MARCSTATE) != 0x41) // check MARCSTATE to see if calibration has finished
    {
        HAL_Delay(1);
    }
    calResults_for_vcdac_start_high[0] = cc1125ReadReg(REG_FS_VCO2);
    calResults_for_vcdac_start_high[1] = cc1125ReadReg(REG_FS_VCO4);
    calResults_for_vcdac_start_high[2] = cc1125ReadReg(REG_FS_CHP); // store these regs
    cc1125WriteReg(REG_FS_VCO2, 0x00); // FS_VCO2 set to 0x00
    cc1125WriteReg(REG_FS_CAL2, original_fs_cal2); // set FS_CAL2 to original
    cc1125WriteStrobe(STROBE_SCAL); // strobe SCAL to start calibration
    while(cc1125ReadReg(REG_MARCSTATE) != MARC_IDLE) // check MARCSTATE to see if calibration has finished
    {
        HAL_Delay(1);
    }
    calResults_for_vcdac_start_mid[0] = cc1125ReadReg(REG_FS_VCO2);
    calResults_for_vcdac_start_mid[1] = cc1125ReadReg(REG_FS_VCO4);
    calResults_for_vcdac_start_mid[2] = cc1125ReadReg(REG_FS_CHP); // store these regs
    // write back higher of the results
    if (calResults_for_vcdac_start_high[0] > calResults_for_vcdac_start_mid[0]) {
        cc1125WriteReg(REG_FS_VCO2,calResults_for_vcdac_start_high[0]);
        cc1125WriteReg(REG_FS_VCO4,calResults_for_vcdac_start_high[1]);
        cc1125WriteReg(REG_FS_CHP,calResults_for_vcdac_start_high[2]);
    } else {
        cc1125WriteReg(REG_FS_VCO2,calResults_for_vcdac_start_mid[0]);
        cc1125WriteReg(REG_FS_VCO4,calResults_for_vcdac_start_mid[1]);
        cc1125WriteReg(REG_FS_CHP,calResults_for_vcdac_start_mid[2]);
    }
}

/* END CC1125 ADVANCED FUNCTIONS */
