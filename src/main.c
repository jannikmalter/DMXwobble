#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "rom/gpio.h"

#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "esp_random.h"
#include "esp_timer.h"

#include <math.h>


// NODE CONFIG

#define BREAK 30
#define MAB 2

#define UNIS 1


uint8_t *DMXbuf;
const char GPIO_patch[UNIS] = {14};    // rack:{16,15,14,13,5,4,2};     mini:{5, 2, 4, 0, 16, 14, 15};
uint_fast8_t DMX_repatch[UNIS] = {0};


uint_fast8_t stopFlag = 0;



void dmx_task()
{

	portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

	uint32_t Cold = 0;

	uint32_t *GPIO_w1ts = (uint32_t *)0x3ff44008;
	uint32_t *GPIO_w1tc = (uint32_t *)0x3ff4400c;
	uint32_t outputH = 0;
	uint32_t outputL = 0;

	uint_fast16_t curbit = 0;
	uint_fast16_t curchan = 0;
	uint_fast8_t chanbit = 0;

	uint_fast8_t i = 0;

	uint_fast8_t chanval = 0;
	uint_fast8_t outbuf = 0;

	uint32_t bitmask = 0;

	bitmask |= 1 << 14;
	bitmask |= 1 << 15;

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = bitmask;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	vPortEnterCritical(&myMutex);
	Cold = xthal_get_ccount();

	
	while (1)
	{
		outbuf = 0b00000000;
		if (curbit < BREAK)
		{
			outbuf = 0b00000000;
		}
		else if (curbit < BREAK + MAB)
		{
			outbuf = 0b11111111;
		}

		else
		{
			curchan = (curbit - BREAK - MAB) / 11;
			chanbit = (curbit - BREAK - MAB) % 11;

			if (chanbit == 0)
				outbuf = 0b00000000;

			else if ((chanbit == 9) || (chanbit == 10))
				outbuf = 0b11111111;

			else
			{
				for (i = 0; i < UNIS; i++)
				{
					if (curchan == 0)
						chanval = 0;
					else
						chanval = DMXbuf[512 * DMX_repatch[i] + curchan - 1];
					if (!((chanval & (1 << (chanbit - 1))) == 0))
						outbuf |= (0b00000001 << i);
				}
			}
		}

		outputH = 0;
		outputL = 0;
		for (i = 0; i < UNIS; i++)
		{
			outputH |= ((1 << i & outbuf) >> i) << GPIO_patch[i];
			outputL |= ((1 << i & ~outbuf) >> i) << GPIO_patch[i];

			outputH |= ((1 << i & ~outbuf) >> i) << (GPIO_patch[i]+1);
			outputL |= ((1 << i & outbuf) >> i) << (GPIO_patch[i]+1);
		}

		while ((xthal_get_ccount() - Cold) < 960)
		{
		};

		*GPIO_w1ts |= outputH;
		*GPIO_w1tc |= outputL;
		Cold += 960;

		curbit++;
		
		if (curbit >= (512 + 1) * 11 + BREAK + MAB)  //450
		{
			curbit = 0;
		}
	}
}


#define NUM_OUT 512        // number of output channels
#define DT_MIN 5000      // shortest fade time [ms]
#define DT_MAX 10000     // longest fade time [ms]
#define DI_MIN 0.1       // minimum intensity change per fade [%]
#define A 1            // dimming curve factor

int64_t* t_start;  // holds start time of current fade for each channel
int64_t* t_stop;   // holds stop time of current fade for each channel
float* i_start;          // holds start intensity of current fade for each channel
float* i_stop;           // holds stop intensity of current fade for each channel

int64_t t;         // holds current time
int64_t dt;        // for measuring time deltas
int64_t length;    // length of a fade
float intensity;         // single intensity value
float di;                // intensity delta of a fade
float prog;              // progress of a fade
int out;       // output value for PWM board

float rnd = 0;


void next(int i){
  rnd = (float)esp_random() / (float)UINT32_MAX;
  length = DT_MIN * 1000L + rnd * (DT_MAX - DT_MIN)*1000L;
  t_start[i] = t;
  t_stop[i] = t + length; 

  // if last fade was up, next one is down
  if (i_stop[i] > i_start[i]){
    rnd = (float)esp_random() / (float)UINT32_MAX;
    intensity = rnd * (i_stop[i] - DI_MIN);
  }
  // if last fade was down, next one is up
  else{
    rnd = (float)esp_random() / (float)UINT32_MAX;
    intensity = (i_stop[i] + DI_MIN) + rnd * (1.0 - (i_stop[i] + DI_MIN));
  }

  i_start[i] = i_stop[i];
  i_stop[i] = intensity;
}

int64_t t1 = 0;
int64_t t2 = 0;
float fps = 0;

void eth_task(){
    while(1){
        for(int i=0; i<NUM_OUT; i++){
            t = esp_timer_get_time();
            length = t_stop[i] - t_start[i];
            dt     =         t - t_start[i];    

            // if fade is over, generate next one
            if (dt >= length){
            next(i);

            }else{      
            di = i_stop[i] - i_start[i];
            prog = (float)dt/(float)length; 
            intensity = i_start[i] + sin(prog*1.57079632679) * di;
            out = (uint8_t)(intensity*255);
			if(i == 0){
				//printf("I= %d\n",out);
			}
            DMXbuf[i] = out;
            }
        }
        vTaskDelay(1);
    }						
}






void app_main(void)
{
	DMXbuf = calloc(UNIS * 512, sizeof(uint8_t));

    t_start = (int64_t*) calloc(NUM_OUT, sizeof(int64_t));
    t_stop = (int64_t*) calloc(NUM_OUT, sizeof(int64_t));
    i_start = (float*) calloc(NUM_OUT, sizeof(float));
    i_stop = (float*) calloc(NUM_OUT, sizeof(float));


	xTaskCreatePinnedToCore(eth_task, "eth_task", 2048, NULL, (tskIDLE_PRIORITY), NULL, 0);
	xTaskCreatePinnedToCore(dmx_task, "dmx_task", 2018, NULL, (configMAX_PRIORITIES - 1), NULL, 1);
}
