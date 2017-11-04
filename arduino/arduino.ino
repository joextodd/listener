extern "C" {
#include "user_interface.h"
#include "i2s_reg.h"
#include "slc_register.h"
#include "esp8266_peri.h"
void rom_i2c_writeReg_Mask(int, int, int, int, int, int);
}

#include <ESP8266WiFi.h>

#define I2SI_PWR        4u
#define I2SI_DATA       12    // I2S data on GPIO12
#define I2SI_BCK        13    // I2S clk on GPIO13
#define I2SI_WS         14    // I2S select on GPIO14

#define SLC_BUF_CNT     4     // Number of buffers in the I2S circular buffer
#define SLC_BUF_LEN     512   // Length of one buffer, in 32-bit words.

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

static sdio_queue_t i2s_slc_items[SLC_BUF_CNT];  // I2S DMA buffer descriptors
static uint32_t *i2s_slc_buf_pntr[SLC_BUF_CNT];  // Pointer to the I2S DMA buffer data
static volatile uint32_t rx_buf_cnt = 0;

void i2s_init();
void slc_init();
void i2s_set_rate(uint32_t rate);
void slc_isr(void *para);

void
setup()
{
  rx_buf_cnt = 0;

  pinMode(I2SI_PWR, OUTPUT);
  pinMode(I2SI_WS, OUTPUT);
  pinMode(I2SI_BCK, OUTPUT);
  pinMode(I2SI_DATA, INPUT);
  digitalWrite(I2SI_PWR, HIGH);

  WiFi.forceSleepBegin();
  delay(500);

  Serial.begin(115200);
  Serial.println("I2S receiver");

  slc_init();
  Serial.println("SLC started");

  i2s_init();
  Serial.println("Initialised");
}

/*
 * TODO: Fix need to wiggle LRCL to start data.
 */
void
loop()
{
  uint32_t cnt;

  if (cnt != rx_buf_cnt) {
    for (int y = 0; y < SLC_BUF_LEN; y++) {
      if (i2s_slc_buf_pntr[rx_buf_cnt][y] > 0) {
        uint32_t value = ~(i2s_slc_buf_pntr[rx_buf_cnt][y]) & 0xFFFFFF;
        uint32_t mask = value & 0x800000;
        int32_t output = ((value & 0xFF0000) >> 16) | (value & 0xFF00) | ((value & 0xFF) << 16);
        if (mask) {
          output = output * -1;
        }
        Serial.println(output);
      }
    }
    cnt = rx_buf_cnt;
  }
}

/*
 * Initialise I2S as a RX master.
 */
void
i2s_init()
{
  // Config RX pin function
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_I2SI_BCK);
//  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);

  // Enable a 160MHz clock
  I2S_CLK_ENABLE();

  // Reset I2S
  I2SC &= ~(I2SRST);
  I2SC |= I2SRST;
  I2SC &= ~(I2SRST);

  // Reset DMA
  I2SFC &= ~(I2SDE | (I2STXFMM << I2STXFM) | (I2SRXFMM << I2SRXFM));

  // Enable DMA
  I2SFC |= I2SDE;

  // Set RX single channel (right)
  I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM));
  I2SCC |= (1 << I2SRXCM);
  i2s_set_rate(96000);

  // Set RX data to be received
  I2SRXEN = SLC_BUF_LEN;

  // Start receiver
  I2SC |= I2SRXS;
}

/*
 * Set I2S clock
 */
void
i2s_set_rate(uint32_t rate)
{
  uint32_t i2s_clock_div = (I2SBASEFREQ / (rate * 32)) & I2SCDM;
  uint8_t i2s_bck_div = (I2SBASEFREQ / (rate * i2s_clock_div)) & I2SBDM;
  Serial.printf("Rate %u Div %u Bck %u Freq %u\n",
  rate, i2s_clock_div, i2s_bck_div, I2SBASEFREQ / (i2s_clock_div * i2s_bck_div));

  // RX master mode, RX MSB shift, right first, msb right
  I2SC &= ~(I2STSM | I2SRSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
  I2SC |= I2SRF | I2SMR | I2SRMS | ((i2s_bck_div - 1) << I2SBD) | ((i2s_clock_div - 1) << I2SCD);
}

/*
 * Initialize the SLC module for DMA operation
 */
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

  // Reset DMA
  ETS_SLC_INTR_DISABLE();
  SLCC0 |= SLCRXLR | SLCTXLR;
  SLCC0 &= ~(SLCRXLR | SLCTXLR);
  SLCIC = 0xFFFFFFFF;

  // Configure DMA
  SLCC0 &= ~(SLCMM << SLCM);      // Clear DMA MODE
  SLCC0 |= (1 << SLCM);           // Set DMA MODE to 1
//  SLCRXDC |= SLCBINR | SLCBTNR;   // Enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE
//  SLCRXDC &= ~(SLCBRXFE | SLCBRXEM | SLCBRXFM);

  // Feed DMA the 1st buffer desc addr
  // To receive data from the I2S slave,
  // counter intuitively we need to use the TXLINK.
  SLCTXL &= ~(SLCTXLAM << SLCTXLA);
  SLCTXL |= (uint32_t)&i2s_slc_items[0] << SLCTXLA;

  ETS_SLC_INTR_ATTACH(slc_isr, NULL);

  // Enable EOF interrupt
  SLCIE = SLCITXEOF;
  ETS_SLC_INTR_ENABLE();

  // Start transmission
  SLCTXL |= SLCTXLS;
}

/*
 * Triggered when SLC has finished writing
 * to one of the buffers.
 */
void
slc_isr(void *para)
{
  uint32_t status;

  status = SLCIS;
  SLCIC = 0xFFFFFFFF;

  if (status == 0) {
    return;
  }

  if (status & SLCITXEOF) {
    // We have received a frame
    ETS_SLC_INTR_DISABLE();
    sdio_queue_t *finished = (sdio_queue_t*)SLCTXEDA;

    // Thanks to cnlohr for the undocumented magic
    finished->owner = 1;
    rx_buf_cnt = (rx_buf_cnt + 1) % SLC_BUF_CNT;
    ETS_SLC_INTR_ENABLE();
  }
}
