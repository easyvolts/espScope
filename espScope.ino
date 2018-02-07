//Created by Valerii Proskurin.
//Date: 30.01.2018


#include <WiFiClient.h>
#include <ESP32WebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <Arduino.h>
#include "driver/ledc.h"
#include "BufferedAdcSampler.h"

//change log:
// v1 - initial version (dma problem?)
// v2 - added mDNS functionality, fixed DMA, code cleanup and rework, adc sampling moved to separate library.
// v3 - added triggers processing (simple digital trigger for auto/single/none/stop mode for rising/falling edge).
//TODO: WebSockets? Check with data generator. Add sampling speed config and time instead of samples on the plot. Voltage instead of ADC on the plot.

/* wifi host id and password */
const char* ssid = "UPC6392915";
const char* password = "GYMEBEHA";
#define DEBUG_LOG_ENABLE

const int XCLK = 32;
const int PCLK = 33;
const int trigIN = 25;

const int D_inputs[10] = {27,17,16,15,14,13,12,4,32,33};


#ifdef DEBUG_LOG_ENABLE
  #define DEBUG_PRINTLN(a) Serial.println(a)
  #define DEBUG_PRINT(a) Serial.print(a)
  #define DEBUG_PRINTLNF(a, f) Serial.println(a, f)
  #define DEBUG_PRINTF(a, f) Serial.print(a, f)
#else
  #define DEBUG_PRINTLN(a)
  #define DEBUG_PRINT(a)
  #define DEBUG_PRINTLNF(a, f)
  #define DEBUG_PRINTF(a, f)
#endif

typedef enum {falling = 0, rising = 1} TRIGGER_EDGE_E;
typedef enum {stopTrig = 0, noneTrig = 1, autoTrig = 2, singleTrig = 3} TRIGGER_MODE_E;
const char * triggerTexts[4] = {"stop\0", "none\0","auto\0","single\0"};
const int triggerEdgeConfig[2] = {FALLING, RISING};
ESP32WebServer server(80);
MDNSResponder mdns;
I2S_AdcSampler adcSampler;
char str[50000] = "stop,0,";
TRIGGER_MODE_E trigger = stopTrig;
TRIGGER_EDGE_E edge = falling;
bool bufferIsFilled = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

bool ClockEnable(int pin, int Hz)
{
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = (ledc_timer_bit_t)1;
    timer_conf.freq_hz = Hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        return false;
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 1;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = pin;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        return false;
    }
    return true;
}

void ClockDisable()
{
    periph_module_disable(PERIPH_LEDC_MODULE);
}


/* my page */
char mainPage[] =
"<!DOCTYPE html>\n\
<html>\n\
  <head>\n\
  <!-- Plotly.js -->\n\
  <script src='https://cdn.plot.ly/plotly-latest.min.js'></script>\n\
  </head>\n\
  \n\
  <body>\n\
      <p style='text-align: left;'><strong>EspScope</strong></p>\n\
      <div id='myDiv'><!-- Plotly chart will be drawn inside this DIV --></div>\n\
      <hr />\n\
      <p style='text-align: left;'>\n\
          Open source <a href='github.com/easyvolts/espScope'>project</a> of esp32 based wireless oscilloscope. \n\
          &nbsp; &nbsp; &nbsp; &nbsp; <b>Trigger mode:</b>\n\
          <input type='radio' id='checkBoxAuto'   name='triggerSelection' value='auto'   onclick='triggerSet(value)' > Auto\n\
          <input type='radio' id='checkBoxSingle' name='triggerSelection' value='single' onclick='triggerSet(value)' > Single\n\
          <input type='radio' id='checkBoxNone'   name='triggerSelection' value='none'   onclick='triggerSet(value)' > None\n\
          <input type='radio' id='checkBoxStop'   name='triggerSelection' value='stop'   onclick='triggerSet(value)' checked='checked'> Stop\n\
		  &nbsp; &nbsp; &nbsp; &nbsp; <b>Trigger edge:</b>\n\
          <input type='radio' id='checkBoxRising'  name='edgeSelection' value='rising'   onclick='edgeSet(value)' > Rising\n\
          <input type='radio' id='checkBoxFalling' name='edgeSelection' value='falling'  onclick='edgeSet(value)' checked='checked'> Falling\n\
      </p>\n\
      <script>\n\
          <!-- JAVASCRIPT CODE GOES HERE -->\n\
          var data = [\n\
              { y: [0,],\n\
                x: [0,],\n\
                type: 'lines+markers' } ];\n\
          var counter = 1;\n\
          var intervalID;\n\
		  var triggerMode = 'none';\n\
          \n\
          Plotly.newPlot('myDiv', data);\n\
          \n\
          function httpGet(theUrl)\n\
          {\n\
              var xmlHttp = new XMLHttpRequest();\n\
              xmlHttp.open( 'GET', theUrl, false ); /*false for synchronous request*/\n\
              xmlHttp.send( null );\n\
              return xmlHttp.responseText;\n\
          }\n\
          \n\
          function plotUpdate() {\n\
                  /*alert('called');*/\n\
                  scopeData = httpGet('/scope');\n\
                  scopeDataArray = scopeData.split(',');\n\
				  triggerMode = scopeDataArray[0]; /*first element in the array is trigger mode*/\n\
				  switch(triggerMode) {\n\
					case 'auto':\n\
						document.getElementById('checkBoxAuto').checked = true;\n\
						break;\n\
					case 'single':\n\
						document.getElementById('checkBoxSingle').checked = true;\n\
						break;\n\
					case 'none':\n\
						document.getElementById('checkBoxNone').checked = true;\n\
						break;\n\
					case 'stop':\n\
						document.getElementById('checkBoxStop').checked = true;\n\
						return;\n\
					default:\n\
				  }\n\
                  /* remove the first trace*/\n\
                  Plotly.deleteTraces('myDiv', 0);\n\
		          scopeDataArray.shift(); /* Removes the first element from scopeDataArray before passing it to plot function*/\n\
                  Plotly.addTraces('myDiv', {y: scopeDataArray});\n\
                  /*Plotly.extendTraces('myDiv', {y: [1,2,3,1]}, [0]);*/\n\
          }\n\
		  \n\
          function triggerSet(trigerStr) {\n\
			if('stop' == trigerStr) {\n\
				clearInterval(document.getElementById('intervalID'));\n\
			} else {\n\
				intervalID = setInterval(plotUpdate, 300);\n\
			}\n\
            httpGet('/'+ trigerStr);\n\
          }\n\
		  \n\
          function edgeSet(edgeStr) {\n\
            httpGet('/'+ edgeStr);\n\
          }\n\
          \n\
      </script>\n\
  </body>\n\
</html>";

void handleRoot() {
    server.send(200, "text/html", mainPage);
}

//trigger interrupt handler (we detect falling edge)
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  detachInterrupt(trigIN);
  adcSampler.stopNonBlocking(1);
  DEBUG_PRINTLN("Trigger detected!");
  portEXIT_CRITICAL_ISR(&mux);
}

/* send data points in responce to "/scope" request */
void handleDataRequest() {
	uint16_t samplePointer = 0;
	int strPointer = 0;
	uint16_t samplesCount = 0;

	//check if we already have the data
	if((I2S_AdcSampler::READY == adcSampler.state()) &&	(trigger != stopTrig))
	{
		//the value before first comma is name of current trigger selected.
		strPointer += sprintf(&str[strPointer], "%s,", triggerTexts[trigger]);
		//trigger fired, we have data in the buffer
		switch (trigger) {
			case autoTrig:
				//buffer is already filled with data
				samplesCount = adcSampler.samplesNumber();
				DEBUG_PRINTLN("Buffer filled");
				break;
			case singleTrig:
				//buffer is already filled with data
				samplesCount = adcSampler.samplesNumber();
				trigger = stopTrig;//stop data acquisition
				DEBUG_PRINTLN("Buffer filled. Stop data acquisition");
				break;
			case noneTrig:
				//buffer is already filled with data
				samplesCount = adcSampler.samplesNumber();
				DEBUG_PRINTLN("Buffer filled");
				break;
			default: //unknown trigger
				samplesCount = 0;
				break;
		}
		//fill the buff with samples converted from bin to str
		for(samplePointer = 0; samplePointer < samplesCount; samplePointer++) {
			strPointer += sprintf(&str[strPointer], "%u,", adcSampler.readSample(samplePointer));
			if(strPointer > sizeof(str)) {
				str[sizeof(str) - 1] = 0;
				break; //to prevent buffer overflow
			}
		}
		//start/restart data acquisition if needed
		if((autoTrig == trigger) || (singleTrig == trigger) || (noneTrig == trigger)) {
			DEBUG_PRINTLN("Repeat data acquisition");
			adcSampler.start(); //start data acquisition if needed
			if(noneTrig == trigger) {
				adcSampler.stopNonBlocking(3); //request acquisition stop after adc buffers are filled
			} else {
				DEBUG_PRINTLN("Wait for the next trigger");
				//GPIO.status_w1tc = BIT(trigIN); //clear interrupt flag
				attachInterrupt(digitalPinToInterrupt(trigIN), handleInterrupt, triggerEdgeConfig[edge]);
			}
		}
	} else {
		if (stopTrig != trigger) {
			if (I2S_AdcSampler::STOPPED == adcSampler.state()) {
				//the value before first comma is name of current trigger selected.
				strPointer += sprintf(&str[strPointer], "%s,", triggerTexts[trigger]);
				//We still waiting for trigger. Add one zero dot for empty plot
				strPointer += sprintf(&str[strPointer], "%s,", "0");
				DEBUG_PRINTLN("Start data acquisition");
				adcSampler.start(); //start data acquisition if needed
				//request acquisition stop after adc buffers are filled
				if(noneTrig == trigger) {
					adcSampler.stopNonBlocking(3);
				} else {
					//GPIO.status_w1tc = BIT(trigIN); //clear interrupt flag
					attachInterrupt(digitalPinToInterrupt(trigIN), handleInterrupt, triggerEdgeConfig[edge]);
				}
			}
		} else {
			//the value before first comma is name of current trigger selected.
			strPointer += sprintf(&str[strPointer], "%s,", triggerTexts[trigger]);
			//We still waiting for trigger. Add one zero dot for empty plot
			strPointer += sprintf(&str[strPointer], "%s,", "0");
		}
	}
	/* respond to the request */
	server.send(200, "text/plain", str);
}

void handleTriggerAutoRequest() {
	DEBUG_PRINTLN("Set AUTO trigger mode");
	trigger = autoTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

void handleTriggerSingleRequest() {
	DEBUG_PRINTLN("Set SINGLE trigger mode");
	trigger = singleTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

void handleTriggerNoneRequest() {
	DEBUG_PRINTLN("Set NONE trigger mode");
	trigger = noneTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

void handleTriggerStopRequest() {
	DEBUG_PRINTLN("Set STOP trigger mode");
	trigger = stopTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

void handleTriggerRisingEdgeRequest() {
	DEBUG_PRINTLN("Set RISING edge trigger mode");
	edge = rising;
	trigger = stopTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

void handleTriggerFallingEdgeRequest() {
	DEBUG_PRINTLN("Set FALLING edge trigger mode");
	edge = falling;
	trigger = stopTrig;
	adcSampler.cancel(); //stop current mode as fast as possible
	server.send(200, "text/plain", "ok");
}

/* cannot handle request so return 404 */
void handleNotFound(){
  String message = "File Not Found\n\n\n\n";
  server.send(404, "text/plain", message);
}

void setup(void){
  Serial.begin(115200);
  Serial.println("Application start");
  WiFi.begin(ssid, password);

  /* indication of connection process */
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //start mDNS service
  if (mdns.begin("espscope",TCPIP_ADAPTER_IF_STA, WiFi.localIP())) {
	  mdns.addService("http","tcp",80);
	  Serial.println("Accessible as \"espscope.local/\" over mDMS service");
  }

  //init ADC sampler
  ClockEnable(XCLK, 40000000); //sampling clock is 40Msps
  adcSampler.init(XCLK, PCLK, D_inputs);
  Serial.println("ADC sampler is ready");

  //init irq for trigger input
  pinMode(trigIN, INPUT_PULLUP);

  //init WebServer handlers
  server.on("/", handleRoot);
  /* this callback handle data request and respond*/
  server.on("/scope", handleDataRequest);
  /* this callback handle single trigger POST message*/
  server.on("/auto", handleTriggerAutoRequest);
  server.on("/single", handleTriggerSingleRequest);
  server.on("/none", handleTriggerNoneRequest);
  server.on("/stop", handleTriggerStopRequest);
  server.on("/falling", handleTriggerFallingEdgeRequest);
  server.on("/rising", handleTriggerRisingEdgeRequest);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop(void){
  server.handleClient();
}
