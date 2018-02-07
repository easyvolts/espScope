//Created by Valerii Proskurin.
//Date: 04.02.2018
//parts of his code are taken from
//https://github.com/igrr/esp32-cam-demo
//by Ivan Grokhotkov

#pragma once

#include <Arduino.h>
#include "rom/lldesc.h"

class I2S_AdcSampler
{
  public:
	typedef enum {
	    STOPPED = 0,
	    BUSY = 1,
		STOPPING = 2,
	    READY = 3
	  } AdcSamplerState_e;

	  I2S_AdcSampler() {
		sys_state = STOPPED;
		i2sInterruptHandle = 0;
	  	memset(dma_descriptor, 0, sizeof(dma_descriptor));
	  	memset(frame, 0, sizeof(frame));
	  	stopCountdown = 0;
	  	lastBuffId = 0;
	  }

	  static bool init(const int XCLK, const int PCLK, const int Din[10]);
	  static void start();
	  static void stop(uint32_t buffCoutdown);
	  static void stopNonBlocking(uint32_t buffCoutdown);
	  static void cancel(void);
	  static void oneFrame();
	  static AdcSamplerState_e state();
	  static uint16_t readSample(uint16_t sampleID);
	  static uint16_t samplesNumber(void);

  private:
	  static uint32_t frame[3][500];
	  static intr_handle_t i2sInterruptHandle;
	  static volatile AdcSamplerState_e sys_state;
	  static volatile uint8_t lastBuffId;
	  static volatile uint8_t stopCountdown;
	  static lldesc_t dma_descriptor[3]; //dma tickTackTock descriptor

	  static void i2sConfReset();
	  static void i2sStop();
	  static void i2sRun();
	  static void IRAM_ATTR i2sInterrupt(void* arg);
	  static bool i2sInit(const int PCLK, const int Din[10]);
	  static void initDmaDescriptors(void);

};
