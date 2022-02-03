#include <Arduino.h>
#include "SPISlave_T4.h"

// 11.7.323 LPSPI4_PCS0_SELECT_INPUT DAISY Register (IOMUXC_LPSPI4_PCS0_SELECT_INPUT) 401F_851Ch (Baseaddr + (_portnum * 0x10))
#define SLAVE_PINS_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x401F84EC + (portnum * 0x10)))

static SPISlave_T4* _LPSPI1 = nullptr;
static SPISlave_T4* _LPSPI3 = nullptr;
static SPISlave_T4* _LPSPI4 = nullptr;

void lpspi1_slave_isr() {
  _LPSPI1->SLAVE_ISR();
}
void lpspi3_slave_isr() {
  _LPSPI3->SLAVE_ISR();
}
void lpspi4_slave_isr() {
  _LPSPI4->SLAVE_ISR();
}

SPISlave_T4::SPISlave_T4(unsigned portnum, SPI_BITS bits) {
  _SPI_ptr isr = nullptr; // Interrupt service routine address
  uint32_t cg = 0; // Clock gate value

  _bits = bits;

  switch(portnum)
  {
    // The order of the Arduino SPI port numbers doesn't correspond to the
    // LPSPI port numbers
    // See e.g.: https://forum.pjrc.com/threads/61234-Teensy-4-1-and-SPI2
    case 0: _LPSPI4 = this; _lpspi = &IMXRT_LPSPI4_S; nvic_irq = IRQ_LPSPI4; isr = lpspi4_slave_isr; cg = CCM_CCGR1_LPSPI4(3); break;
    case 1: _LPSPI3 = this; _lpspi = &IMXRT_LPSPI3_S; nvic_irq = IRQ_LPSPI3; isr = lpspi3_slave_isr; cg = CCM_CCGR1_LPSPI3(3); break;
    case 2: _LPSPI1 = this; _lpspi = &IMXRT_LPSPI1_S; nvic_irq = IRQ_LPSPI1; isr = lpspi1_slave_isr; cg = CCM_CCGR1_LPSPI1(3); break;
  }

  // Clock gating register, see 14.7.22 p1085
  // Oddly, changing from CG3 to CG2 causes hang on init.
  // There appears to be a need for coordination between setting this register and CCM_CBCMR, as per the thread:
  // https://forum.pjrc.com/threads/59254-SPI-Slave-Mode-on-Teensy-4
  // "CCM_CBCMR |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON); //Clock reaktivieren (reactivate)". Note he also appears to
  // set SION on the CS pin as we also did below.
  CCM_CCGR1 |= cg;

  attachInterruptVector(nvic_irq, isr);

  /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
  SLAVE_PINS_ADDR; // TODO: tidy up for ports other than LPSPI4
  spiAddr[0] = 0; // IOMUXC_LPSPIx_PCS0_SELECT_INPUT. For LSPCI4: (401F_851Ch) 0=GPIO_B0_00_ALT3 1=GPIO_B1_04_ALT1
  spiAddr[1] = 0; // IOMUXC_LPSPIx_SCK_SELECT_INPUT . For LSPCI4: (401F_8520h) 0=GPIO_B0_03_ALT3 1=GPIO_B1_07_ALT1
  spiAddr[2] = 0; // IOMUXC_LPSPIx_SDI_SELECT_INPUT . For LSPCI4: (401F_8524h) 0=GPIO_B0_02_ALT3 1=GPIO_B1_05_ALT1
  spiAddr[3] = 0; // IOMUXC_LPSPIx_SDO_SELECT_INPUT . For LSPCI4: (401F_8528h) 0=GPIO_B0_02_ALT3 1=GPIO_B1_06_ALT1

  // These are the primary SPI mux control registers.
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) 13 ALT3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) 12 ALT3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) 11 ALT3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) 10 ALT3

  // These are the primary SPI2 mux control registers (unverified -- JG).
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 0x17; // LPSPI3_PCS0 (CS1) 0 ALT7 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 0x12; // LPSPI3_SCK1 (CLK) 27 ALT2
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 0x17; // LPSPI3_SDI (MISO1) 1 ALT7
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 0x12; // LPSPI3_SDO (MOSI1) 26 ALT2

  // These are the primary SPI3 mux control registers (unverified -- JG).
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 0x14; // LPSPI2_PCS0 (CS2) 36 ALT4 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 0x14; // LPSPI2_SCK1 (CLK2) 37 ALT4
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x14; // LPSPI2_SDI (MISO2) 34 ALT4
  // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 0x14; // LPSPI2_SDO (MOSI2) 35 ALT4
}


// "disables the slave output pin so it listens to traffic only." See: https://forum.pjrc.com/threads/66389-SPISlave_T4
void SPISlave_T4::swapPins(bool enable) {
  // Disable Module
  _lpspi->CR &= ~LPSPI_CR_MEN;

  // CFGR1[PINCFG] = 3: swap SDI/SDO, 2: Single pin on SDO, 1: Single pin on SDI, 0: Normal SDI/SDO
  _lpspi->CFGR1 = (_lpspi->CFGR1 & ~(LPSPI_CFGR1_PINCFG(3))) | (enable) ? LPSPI_CFGR1_PINCFG(3) : LPSPI_CFGR1_PINCFG(0);

  // Enable Module
  _lpspi->CR |= LPSPI_CR_MEN;

  if ( sniffer_enabled ) sniffer();
}


void SPISlave_T4::sniffer(bool enable) {
  sniffer_enabled = enable;
  if ( _LPSPI4 == this ) {
    if ( sniffer_enabled ) {
      if ( (_lpspi->CFGR1 & LPSPI_CFGR1_PINCFG(3)) != 0) { /* if pins are swapped */
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3;  // LPSPI4 SCK (CLK) 13 ALT3
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x13; // LPSPI_SDI (MISO) 12 ALT3+SION Not sure, see 11.7.76 p509 (JG) // 0; // LPSPI4 SDI (MISO) 12 ALT0 LCD_ENABLE? See comment above swapPins()
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3;  // LPSPI4 SDO (MOSI) 11 ALT3
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3;  // LPSPI4 PCS0 (CS) 10 ALT3

        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 0x14; // LPSPI2_PCS0 (CS2) 36 ALT4 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 0x14; // LPSPI2_SCK1 (CLK2) 37 ALT4 + SION
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x14; // LPSPI2_SDI (MISO2) 34 ALT4 + SION
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 0x14; // LPSPI2_SDO (MOSI2) 35 ALT4 + SION
      }
      else { // pins normal
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) 13 ALT3
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) 12 ALT3
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) 11 ALT3 Not sure, see 11.7.77 p510 (JG) // 0; // LPSPI4 SDO (MOSI) 11 ALT0 LCD_HSYNC? See comment above swapPins()
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) 10 ALT3

        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 0x14; // LPSPI2_PCS0 (CS2) 36 ALT4 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 0x14; // LPSPI2_SCK1 (CLK2) 37 ALT4 + SION
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x14; // LPSPI2_SDI (MISO2) 34 ALT4 + SION
        // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 0x14; // LPSPI2_SDO (MOSI2) 35 ALT4 + SION
      }
    }
    else { // sniffer disabled
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; // LPSPI4 SCK (CLK) ALT3
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; // LPSPI4 SDI (MISO) ALT3
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; // LPSPI4 SDO (MOSI) ALT3
      IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; // LPSPI4 PCS0 (CS) ALT3
      
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = 0x14; // LPSPI2_PCS0 (CS2) 36 ALT4 + SION. See: https://forum.pjrc.com/archive/index.php/t-59893.html
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = 0x14; // LPSPI2_SCK1 (CLK2) 37 ALT4 + SION
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x14; // LPSPI2_SDI (MISO2) 34 ALT4 + SION
      // IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = 0x14; // LPSPI2_SDO (MOSI2) 35 ALT4 + SION
    }
  }
}


bool SPISlave_T4::active() {
  return !(_lpspi->SR & LPSPI_SR_FCF); // returns 1 if frame transfer not completed
}


bool SPISlave_T4::available() {
  return !(_lpspi->RSR & LPSPI_RSR_RXEMPTY); // returns 1 if Rx FIFO is not empty
}


void SPISlave_T4::pushr(uint32_t data) {
  _lpspi->TDR = data;
}


uint32_t SPISlave_T4::popr() {
  uint32_t data = _lpspi->RDR;
  //_lpspi->SR = LPSPI_SR_WCF; /* Clear WCF */
  return data;
}


void SPISlave_T4::SLAVE_ISR() {
  if ( _spihandler ) {
    _spihandler();
    _lpspi->SR = LPSPI_SR_DMF | LPSPI_SR_REF | LPSPI_SR_TEF | LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
    asm volatile ("dsb");
    return;
  }

  while ( available() ) {
    if ( _lpspi->SR & LPSPI_SR_TEF ) { // transmit error, clear flag, check cabling
      _lpspi->SR = LPSPI_SR_TEF;
      transmit_errors++;
    }
    uint32_t val = popr();
    Serial.print(val, HEX); Serial.print(" ");
  }
  Serial.println();
  _lpspi->SR = LPSPI_SR_DMF | LPSPI_SR_REF | LPSPI_SR_TEF | LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
  asm volatile ("dsb");
}


void SPISlave_T4::begin() {
  _lpspi->CR = LPSPI_CR_RST;// Reset Module
  _lpspi->CR = 0;           // Disable Module
  _lpspi->FCR = LPSPI_FCR_RXWATER(0) | LPSPI_FCR_TXWATER(0); // x10001; // Watermark for RX and TX
  _lpspi->IER = LPSPI_IER_RDIE; // RX Interrupt
  _lpspi->CFGR0 = 0;        // Verify HRSEL. Should be 1?
  _lpspi->CFGR1 = 0;        // slave, sample on SCK rising edge, !autoPCS (must raise CS between frames), FIFO will stall, CS active low, match disabled,
  _lpspi->CR |= LPSPI_CR_MEN /*| LPSPI_CR_DBGEN*/; /* Enable Module, Debug Mode */
  _lpspi->SR = LPSPI_SR_DMF | LPSPI_SR_REF | LPSPI_SR_TEF | LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF; // Clear status register
  _lpspi->TCR = LPSPI_TCR_FRAMESZ(_bits - 1); // TODO: CPOL/CPHA/LSBF
  _lpspi->TDR = 0x0;        // dummy data, must populate initial TX slot
  NVIC_ENABLE_IRQ(nvic_irq);
  NVIC_SET_PRIORITY(nvic_irq, 1);
}
