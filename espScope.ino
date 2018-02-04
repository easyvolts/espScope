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
//TODO: trigger processing. Single and Auto triggering logic. WebSockets?

#define SAMPLES_COUNT 1000
/* wifi host id and password */
const char* ssid = "UPC6392915";
const char* password = "GYMEBEHA";
#define DEBUG_LOG_ENABLE

const int XCLK = 32;
const int PCLK = 33;

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

ESP32WebServer server(80);
MDNSResponder mdns;
I2S_AdcSampler adcSampler;

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
          Open source <a href='github.com/easyvolts/espScope'>project</a> of esp32 based wireless oscilloscope. &nbsp; &nbsp; &nbsp; &nbsp;\n\
          Trigger mode:\n\
          <input type='radio' name='triggerSelection' value='auto'> Auto\n\
          <input type='radio' name='triggerSelection' value='single'> Single\n\
          <input type='radio' name='triggerSelection' value='none'  onclick='startUpdate()'> None\n\
          <input type='radio' name='triggerSelection' value='stop'  onclick='stopUpdate()' checked='checked'> Stop\n\
      </p>\n\
      <script>\n\
          <!-- JAVASCRIPT CODE GOES HERE -->\n\
          var data = [\n\
              { y: [0,0,0,0],\n\
                x: [0,1,2,3],\n\
                type: 'lines+markers' } ];\n\
          var counter = 1;\n\
          var intervalID;\n\
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
                  scopeData = httpGet('/scope?');\n\
                  /* remove the first trace*/\n\
                  Plotly.deleteTraces('myDiv', 0);\n\
                  scopeDataArray = scopeData.split(',');\n\
                  Plotly.addTraces('myDiv', {y: scopeDataArray});\n\
                  /*Plotly.extendTraces('myDiv', {y: [1,2,3,1]}, [0]);*/\n\
          }\n\
          function startUpdate() {\n\
            intervalID = setInterval(plotUpdate, 300);\n\
          }\n\
          function stopUpdate() {\n\
            clearInterval(intervalID);\n\
          }\n\
          \n\
      </script>\n\
  </body>\n\
</html>";

void handleRoot() {
    server.send(200, "text/html", mainPage);
}

/* send data points in responce to "/scope?" request */
char str[50000];
void handleDataRequest() {
  uint16_t samplePointer = 0;
  int strPointer = 0;
  DEBUG_PRINT("Answer to the request. Samples count =");
  DEBUG_PRINTLN(adcSampler.samplesNumber());
  adcSampler.oneFrame();
  //convert bin data to str
  for(samplePointer = 0; samplePointer < adcSampler.samplesNumber(); samplePointer++) {
    strPointer += sprintf(&str[strPointer], "%u,", adcSampler.readSample(samplePointer));
    if(strPointer > sizeof(str)) {
    	str[sizeof(str) - 1] = 0;
    	break; //to prevent buffer overflow
    }
  }
  //char str[] = "1,12,24,23,33,2"; //send data bytes only
  /* respond to the request */
  server.send(200, "text/plain", str);
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
  /* main page handler */
  server.on("/", handleRoot);
  /* this callback handle GPIO request and respond*/
  server.on("/scope", handleDataRequest);

  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("HTTP server started");
}

void loop(void){
  server.handleClient();
}
