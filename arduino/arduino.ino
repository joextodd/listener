extern "C" {
#include "user_interface.h"
#include "i2s_reg.h"
#include "slc_register.h"
#include "esp8266_peri.h"
void rom_i2c_writeReg_Mask(int, int, int, int, int, int);
}

#define I2SI_DATA       12
#define I2SI_BCK        13
#define I2SI_WS         14

#define SLC_BUF_CNT (4)    // Number of buffers in the I2S circular buffer
#define SLC_BUF_LEN (512)  // Length of one buffer, in 32-bit words.

typedef struct {
  uint32_t blocksize:12;
  uint32_t datalen:12;
  uint32_t unused:5;
  uint32_t sub_sof:1;
  uint32_t eof:1;
  uint32_t owner:1;

  uint32_t buf_ptr;
  uint32_t next_link_ptr;
} sdio_queue_t;

static volatile uint32_t i2s_slc_queue[SLC_BUF_CNT - 1];
static volatile uint8_t i2s_slc_queue_len;
static uint32_t *i2s_slc_buf_pntr[SLC_BUF_CNT];  // Pointer to the I2S DMA buffer data
static sdio_queue_t i2s_slc_items[SLC_BUF_CNT]; // I2S DMA buffer descriptors
static volatile uint32_t rx_buf_cnt = 0;

sdio_queue_t i2s_rx_queue1, i2s_rx_queue2, i2s_rx_queue3;

void i2s_init();
void slc_init();
void i2s_set_rate(uint32_t rate);
void slc_isr(void *para);

void
setup()
{
  pinMode(I2SI_WS, OUTPUT);
  pinMode(I2SI_BCK, OUTPUT);
  pinMode(I2SI_DATA, INPUT);

  Serial.begin(115200);
  Serial.println("I2S receiver");

  slc_init();
  Serial.println("SLC started");

  i2s_init();
  Serial.println("Initialised");
}

void
loop()
{
   delay(1000);
   Serial.println(rx_buf_cnt);
   rx_buf_cnt = 0;
}

void
i2s_init()
{
  // Config RX pin function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

  // Enable a 160MHz clock
  I2S_CLK_ENABLE();

  // Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  // Set RX FIFO_MOD = 0 and disable DMA (FIFO only)
  I2SFC &= ~(I2SDE | (I2SRXFMM << I2SRXFM));
  // Enable DMA
  I2SFC |= I2SDE;

  // Set RX CHAN_MOD = 0
  I2SCC &= ~(I2SRXCMM << I2SRXCM);
  i2s_set_rate(44100);

  // Start receiver
  I2SC |= I2SRXS;
}

void
i2s_set_rate(uint32_t rate)
{
  uint32_t i2s_clock_div = (I2SBASEFREQ / (rate * 32)) & I2SCDM;
  uint8_t i2s_bck_div = (I2SBASEFREQ / (rate * i2s_clock_div * 2)) & I2SBDM;
  Serial.printf("Rate %u Div %u Bck %u Freq %u\n", 
  rate, i2s_clock_div, i2s_bck_div, I2SBASEFREQ / (i2s_clock_div * i2s_bck_div * 2));

  //!trans master, !bits mod, rece slave mod, rece msb shift, right first, msb right
  I2SC &= ~(I2STSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  I2SC |= I2SRF | I2SMR | I2SRMS | (24 << I2SBD) | (32 << I2SCD);
}

// Initialize the SLC module for DMA function
// https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_i2s.c
void
slc_init()
{
  for (int x = 0; x < SLC_BUF_CNT; x++) {
    i2s_slc_buf_pntr[x] = (uint32_t *)malloc(SLC_BUF_LEN * 4);
    for (int y = 0; y < SLC_BUF_LEN; y++) i2s_slc_buf_pntr[x][y] = 0;

    i2s_slc_items[x].unused = 0;
    i2s_slc_items[x].owner = 1;
    i2s_slc_items[x].eof = 1;
    i2s_slc_items[x].sub_sof = 0;
    i2s_slc_items[x].datalen = SLC_BUF_LEN * 4;
    i2s_slc_items[x].blocksize = SLC_BUF_LEN * 4;
    i2s_slc_items[x].buf_ptr = (uint32_t)&i2s_slc_buf_pntr[x][0];
    i2s_slc_items[x].next_link_ptr = (int)((x < (SLC_BUF_CNT - 1)) ? (&i2s_slc_items[x + 1]) : (&i2s_slc_items[0]));
  }

  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  // Configure DMA
  SLCC0 &= ~(SLCMM << SLCM);      // Clear DMA MODE
  SLCC0 |= (1 << SLCM);           // Set DMA MODE to 1
  SLCRXDC |= SLCBINR | SLCBTNR;   // Enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE
  SLCRXDC &= ~(SLCBRXFE | SLCBRXEM | SLCBRXFM);  // disable RX_FILL, RX_EOF_MODE and RX_FILL_MODE

  // Feed DMA the 1st buffer desc addr
  // To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
  // expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
  // an error at us otherwise. Just feed it any random descriptor.
  SLCTXL &= ~(SLCTXLAM << SLCTXLA);  // clear TX descriptor address
  SLCTXL |= (uint32_t)&i2s_slc_items[0] << SLCTXLA;  // set TX descriptor address. any random desc is OK, we don't use TX but it needs to be valid

  ETS_SLC_INTR_ATTACH(slc_isr, NULL);
  
  // Enable EOF interrupt
  SLCIE = SLCITXEOF;  
  ETS_SLC_INTR_ENABLE();

  // Start transmission
  SLCTXL |= SLCTXLS;
}

void
slc_isr(void *para)
{
  uint32_t slc_intr_status;
  
  slc_intr_status = SLCIS;
  SLCIC = 0xFFFFFFFF;
  
  if (slc_intr_status == 0) {
    return;
  }

  if (slc_intr_status & SLCITXEOF) {
    ETS_SLC_INTR_DISABLE();
    sdio_queue_t *finished_item = (sdio_queue_t*)SLCTXEDA;
    i2s_slc_queue[i2s_slc_queue_len] = finished_item->buf_ptr;
    // Thanks to cnlohr for the undocumented magic
    finished_item->owner = 1;
    rx_buf_cnt++;
    ETS_SLC_INTR_ENABLE();
  }
}
