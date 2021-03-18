/*
 * ADC_4_Channel.h
 *
 * Created: 2020-04-30 오후 10:19:23
 *  Author: khs
 */ 


#ifndef ADC_4_CHANNEL_H_
#define ADC_4_CHANNEL_H_

#include <asf.h>
#include <custom_pin_map.h>

#define RESOLUTION_ADC			4096
#define VREF_mV					2027
#define REVERSE_GAIN			2
// adc spec data
#define PINMUX_POSITION			1			// analog perip
// pin mux
#define SENSOR_STABLE_TIME		2000		// 2000 mSec


struct adc_module adc_instance;
// adc module handle
struct port_config pin_conf;
// generic config of out pins


void config_adc_func(void);
void get_sen_data(uint16_t* ary_4);
void disable_adc(struct adc_module* adc_instance);
void sensor_on(void);
void sensor_off(void);

#endif /* ADC_4_CHANNEL_H_ */