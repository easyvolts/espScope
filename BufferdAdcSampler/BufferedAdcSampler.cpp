//Created by Valerii Proskurin.
//Date: 04.02.2018
//parts of his code are taken from
//https://github.com/igrr/esp32-cam-demo
//by Ivan Grokhotkov

#include "BufferedAdcSampler.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

#define ESP_LOGD(...)

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

//private variables
intr_handle_t I2S_AdcSampler::i2sInterruptHandle;
volatile I2S_AdcSampler::AdcSamplerState_e I2S_AdcSampler::sys_state;
volatile uint8_t I2S_AdcSampler::stopCountdown;
volatile uint8_t I2S_AdcSampler::lastBuffId;
lldesc_t I2S_AdcSampler::dma_descriptor[3];
uint32_t I2S_AdcSampler::frame[3][500]; //buffer before+at+after trigger (can be up to 4096 bytes each)

void I2S_AdcSampler::initDmaDescriptors(void)
{
	//init Tick (buff before trigger) stage of DMA
	dma_descriptor[0].length = 0; //number of byte written to the buffer
	dma_descriptor[0].size = sizeof(frame[0]); //max size of the buffer in bytes
	dma_descriptor[0].owner = 1;
	dma_descriptor[0].sosf = 1;
	dma_descriptor[0].buf = (uint8_t*)&frame[0][0];
	dma_descriptor[0].offset = 0;
	dma_descriptor[0].empty = 0;
	dma_descriptor[0].eof = 0;
	//pointer to the next descriptor
	dma_descriptor[0].qe.stqe_next = &dma_descriptor[1];

	//init Tack (buff at trigger) stage of DMA
	dma_descriptor[1].length = 0; //number of byte written to the buffer
	dma_descriptor[1].size = sizeof(frame[1]); //max size of the buffer in bytes
	dma_descriptor[1].owner = 1;
	dma_descriptor[1].sosf = 1;
	dma_descriptor[1].buf = (uint8_t*)&frame[1][0];
	dma_descriptor[1].offset = 0;
	dma_descriptor[1].empty = 0;
	dma_descriptor[1].eof = 0;
	//pointer to the next descriptor
	dma_descriptor[1].qe.stqe_next = &dma_descriptor[2]; //we do a circular buffer

	//init Tock (buff after trigger) stage of DMA
	dma_descriptor[2].length = 0; //number of byte written to the buffer
	dma_descriptor[2].size = sizeof(frame[2]); //max size of the buffer in bytes
	dma_descriptor[2].owner = 1;
	dma_descriptor[2].sosf = 1;
	dma_descriptor[2].buf = (uint8_t*)&frame[2][0];
	dma_descriptor[2].offset = 0;
	dma_descriptor[2].empty = 0;
	dma_descriptor[2].eof = 0;
	//pointer to the next descriptor
	dma_descriptor[2].qe.stqe_next = &dma_descriptor[0]; //we do a circular buffer
};


void I2S_AdcSampler::i2sConfReset()
{
  const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
  I2S0.lc_conf.val |= lc_conf_reset_flags;
  I2S0.lc_conf.val &= ~lc_conf_reset_flags;

  const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
  I2S0.conf.val |= conf_reset_flags;
  I2S0.conf.val &= ~conf_reset_flags;
  while (I2S0.state.rx_fifo_reset_back);
}

void I2S_AdcSampler::start()
{
	ESP_LOGD("Sampling started");
	i2sRun();
}
//we have 3 buffers that a closed in a single circular buffer.
//we can select number of buffers recorded after stop command.
//For example: buffCoutdown=0 -> we have 2 buffers before STOP and 1 recorded during STOP;
// buffCoutdown=1 -> we have 1 buffers before STOP, 1 recorded during STOP and 1 after;
// buffCoutdown=2 -> we have 0 buffers before STOP, 1 recorded during STOP and 2 after;
// buffCoutdown=3 -> we have 0 buffers before STOP, 0 recorded during STOP and 3 after;
void I2S_AdcSampler::stop(uint32_t buffCoutdown)
{
	if(BUSY != sys_state) return; //ignore stop request for inappropriate state

	if(buffCoutdown > 3) buffCoutdown = 3;
	stopCountdown = buffCoutdown;
	sys_state = STOPPING;
	while(READY != sys_state);
	ESP_LOGD("Sampling stopped");
}

//end of sampling must be checked by user (using sys_state)
void I2S_AdcSampler::stopNonBlocking(uint32_t buffCoutdown)
{
	if(BUSY != sys_state) return; //ignore stop request for inappropriate state

	if(buffCoutdown > 3) buffCoutdown = 3;
	stopCountdown = buffCoutdown;
	sys_state = STOPPING;
}

//cancel data acquisition
void I2S_AdcSampler::cancel(void)
{
	i2sStop();
	sys_state = STOPPED;
}

void I2S_AdcSampler::oneFrame()
{
  start();
  stop(3);
}

void IRAM_ATTR I2S_AdcSampler::i2sInterrupt(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;

    ESP_LOGD("I2S irq handler");
    //stop i2s
    if(STOPPING == sys_state)
    {
    	if(stopCountdown) {
    		stopCountdown--; //we record until end of current buffer and then one more buffer.
    		return;
    	}
		i2sStop();
		sys_state = READY;
    }
    lastBuffId = (lastBuffId + 1)%3;
}

uint16_t I2S_AdcSampler::readSample(uint16_t sampleID)
{
	uint32_t * samplePointer = NULL;
	uint8_t buffId = 0;
	uint16_t wordID = sampleID/2;
	const uint16_t singleBuffSizeInWords = sizeof(frame[0])/4;

	if(wordID >= 2*singleBuffSizeInWords) {
		buffId = lastBuffId;
	} else if(wordID >= 1*singleBuffSizeInWords) {
		buffId = (lastBuffId+2)%3;
	} else {
		buffId = (lastBuffId+1)%3;
	}
	samplePointer = &frame[buffId][0] + wordID % singleBuffSizeInWords;

	if(sampleID%2) {
		return (uint16_t)*samplePointer;
	} else {
		return (*samplePointer)>>16;
	}
}

I2S_AdcSampler::AdcSamplerState_e I2S_AdcSampler::state(){
	return sys_state;
}

uint16_t I2S_AdcSampler::samplesNumber(void){
	return (sizeof(frame)/2);
}

void I2S_AdcSampler::i2sStop()
{
    ESP_LOGD("I2S Stop");
    esp_intr_disable(i2sInterruptHandle);
    i2sConfReset();
    I2S0.conf.rx_start = 0;
}

void I2S_AdcSampler::i2sRun()
{
    ESP_LOGD("I2S Run");
    esp_intr_disable(i2sInterruptHandle);
    sys_state = BUSY;
    lastBuffId = 0;
    i2sConfReset();
    I2S0.rx_eof_num = 0xFFFFFFFF;
    I2S0.in_link.addr = (uint32_t)&dma_descriptor[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;
    esp_intr_enable(i2sInterruptHandle);
    I2S0.conf.rx_start = 1;
}

//we record 10bit samples
bool I2S_AdcSampler::init(const int XCLK, const int PCLK, const int Din[10])
{
	initDmaDescriptors();
	i2sInit(PCLK, Din);
	return true;
}

bool I2S_AdcSampler::i2sInit(const int PCLK, const int Din[10])
{
  int pins[] = {PCLK, Din[0], Din[1], Din[2], Din[3], Din[4], Din[5], Din[6], Din[7], Din[8], Din[9]};
  gpio_config_t conf = {
    .pin_bit_mask = 0,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix, last parameter is invert
    gpio_matrix_in(Din[0],    I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(Din[1],    I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(Din[2],    I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(Din[3],    I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(Din[4],    I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(Din[5],    I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(Din[6],    I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(Din[7],    I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(Din[8],    I2S0I_DATA_IN8_IDX, false);
    gpio_matrix_in(Din[9],    I2S0I_DATA_IN9_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN10_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN11_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN12_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN13_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN14_IDX, false);
    gpio_matrix_in(0x30,  I2S0I_DATA_IN15_IDX, false);

    //for i2s in parallel camera input mode data is receiver only when V_SYNC = H_SYNC = H_ENABLE = 1. We don't use these inputs so simply set them High
    gpio_matrix_in(0x38, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);  //0x30 sends 0, 0x38 sends 1
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(PCLK, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register
    i2sConfReset();
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    //two bytes per dword packing
    I2S0.fifo_conf.rx_fifo_mod = SM_0A0B_0C0D;  //pack two bytes in one UINT32
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM, &i2sInterrupt, NULL, &i2sInterruptHandle);
    return true;
}
