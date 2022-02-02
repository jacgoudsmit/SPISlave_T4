#if !defined(_SPISlave_T4_H_)
#define _SPISlave_T4_H_

#include <SPI.h>

typedef enum SPI_BITS {
  SPI_8_BITS = 8,
  SPI_16_BITS = 16,
  SPI_32_BITS = 32,
} SPI_BITS;

typedef void (*_SPI_ptr)();

extern SPIClass SPI;

class SPISlave_T4_Base {
  public:
    virtual void SLAVE_ISR();
};

class SPISlave_T4 : public SPISlave_T4_Base {
  public:
    SPISlave_T4(SPIClass* port, SPI_BITS bits);
    void begin();
    uint32_t transmitErrors();
    void onReceive(_SPI_ptr handler) { _spihandler = handler; }
    bool active();
    bool available();
    void sniffer(bool enable = 1);
    void swapPins(bool enable = 1);
    void pushr(uint32_t data);
    uint32_t popr();

  private:
    SPIClass *_port;
    SPI_BITS _bits;
    _SPI_ptr _spihandler = nullptr;
    void SLAVE_ISR();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    uint32_t transmit_errors = 0;
    bool sniffer_enabled = 0;
};

#endif