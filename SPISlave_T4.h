#if !defined(_SPISlave_T4_H_)
#define _SPISlave_T4_H_

#include <SPI.h>

typedef enum SPI_BITS {
  SPI_8_BITS = 8,
  SPI_16_BITS = 16,
  SPI_32_BITS = 32,
} SPI_BITS;

typedef void (*_SPI_ptr)();

class SPISlave_T4 {
  public:
    SPISlave_T4(unsigned portnum, SPI_BITS bits);
    void begin(uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0);

    uint32_t transmitErrors() { return transmit_errors; }
    void onReceive(_SPI_ptr handler) { _spihandler = handler; }
    bool active();
    bool available();
    void sniffer(bool enable = 1);
    void swapPins(bool enable = 1);
    void pushr(uint32_t data);
    uint32_t popr();

  protected:
    IMXRT_LPSPI_t *_lpspi; // Book chapter 48; p2799
    SPI_BITS _bits;
    _SPI_ptr _spihandler = nullptr;
    virtual void SLAVE_ISR();
    IRQ_NUMBER_t nvic_irq;
    uint32_t transmit_errors = 0;
    bool sniffer_enabled = 0;

  friend void lpspi4_slave_isr();
  friend void lpspi3_slave_isr();
  friend void lpspi1_slave_isr();
};

#endif